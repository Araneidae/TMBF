#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <fts.h>
#include <errno.h>

#include <rsrv.h>       // casStatsFetch

#include "error.h"
#include "epics_device.h"
#include "epics_extra.h"
#include "hardware.h"

#include "sensors.h"



/* We poll the sensors every 5 seconds.  This interval is a tradeoff between
 * reported precision and timeliness. */
#define SENSORS_POLL_INTERVAL   5

/* Some scaling factors. */
#define SCALE_K     1024
#define SCALE_M     (SCALE_K * SCALE_K)


/* Sensor variables. */

static double LastUptime;
static double LastIdle;
static double EpicsStarted;

static double UptimeHours; // Machine uptime in hours
static double EpicsUpHours; // EPICS run time in hours
static double CpuUsage;    // % CPU usage over the last sample interval

static double MemoryFree;  // Nominal memory free (free + cached - ramfs) in MB
static double RamfsUsage;  // Number of MB allocated in RAM filesystems

static int MbTemperature;
static int FanSpeeds[2];


/* Hard-wired paths to temperature and fan sensors. */
#define I2C_DEVICE  "/sys/bus/i2c/devices/"
static const char *proc_temp_mb  = I2C_DEVICE "0-0029/temp1_input";
static const char *proc_fan0     = I2C_DEVICE "0-004b/fan1_input";
static const char *proc_fan1     = I2C_DEVICE "0-0048/fan1_input";
static const char *proc_fan0_set = I2C_DEVICE "0-004b/fan1_target";
static const char *proc_fan1_set = I2C_DEVICE "0-0048/fan1_target";
static const char *proc_fan0_pwm = I2C_DEVICE "0-004b/pwm1_enable";
static const char *proc_fan1_pwm = I2C_DEVICE "0-0048/pwm1_enable";

static char **RamFileSystems;      // List of file systems to scan for files


/* Helper routine for using scanf to parse a file. */
static bool parse_file(
    const char *Filename, int Count, const char *Format, ...)
    __attribute__((format(scanf, 3, 4)));
static bool parse_file(
    const char *Filename, int Count, const char *Format, ...)
{
    FILE *input;
    bool Ok = TEST_NULL(input = fopen(Filename, "r"));
    if (Ok)
    {
        va_list args;
        va_start(args, Format);
        Ok = TEST_OK(vfscanf(input, Format, args) == Count);
        fclose(input);
    }
    return Ok;
}



/* Total uptime and idle time can be read directly from /proc/uptime, and by
 * keeping track of the cumulative idle time we can report percentage CPU
 * usage over the scan period. */
static void ProcessUptimeAndIdle(void)
{
    double Uptime, Idle;
    if (parse_file("/proc/uptime", 2, "%lf %lf", &Uptime, &Idle))
    {
        double SampleTime = Uptime - LastUptime;
        double IdleTime = Idle - LastIdle;
        CpuUsage = 1e2 * (1.0 - IdleTime / SampleTime);

        LastUptime = Uptime;
        LastIdle = Idle;

        UptimeHours = Uptime / 3600.;
        EpicsUpHours = (Uptime - EpicsStarted) / 3600.;
    }
}


static void InitialiseUptime(void)
{
    parse_file("/proc/uptime", 1, "%lf", &EpicsStarted);
}


static bool InitialiseRamfsUsage(void)
{
    const char *string;
    bool Ok = TEST_NULL(string = getenv("TEMP_FS_LIST"));
    if (Ok)
    {
        char *ramfs_list = strdup(string);

        /* Split the string up and count the number of entries. */
        unsigned int ramfs_count = 1;
        for (char *ramfs = strchr(ramfs_list, ' '); ramfs != NULL;
             ramfs = strchr(ramfs, ' '))
        {
            *ramfs++ = '\0';
            ramfs_count += 1;
        }

        /* Assemble the final list of file systems for fts_... to scan. */
        RamFileSystems = malloc((ramfs_count + 1) * sizeof(char *));
        for (unsigned int i = 0; i < ramfs_count; i ++)
        {
            RamFileSystems[i] = ramfs_list;
            ramfs_list += strlen(ramfs_list) + 1;
        }
        RamFileSystems[ramfs_count] = NULL;
    }
    return Ok;
}


/* This discovers how many bytes of space are being consumed by the ramfs:
 * this needs to be subtracted from the "cached" space.
 *
 * We do this by walking all of the file systems mounted as ramfs -- the
 * actual set of mount points must be set in TEMP_FS_LIST. */
static int FindRamfsUsage(void)
{
    FTS *fts;
    if (TEST_NULL(fts = fts_open(
            RamFileSystems, FTS_PHYSICAL | FTS_XDEV, NULL)))
    {
        int total = 0;
        FTSENT *ftsent;
        while (ftsent = fts_read(fts),  ftsent != NULL)
            if (ftsent->fts_info != FTS_D)
                total += ftsent->fts_statp->st_size;
        fts_close(fts);
        return total;
    }
    else
        return 0;
}



/* This helper routine is used to read a specific line from the /proc/meminfo
 * file: it scans for a line of the form
 *      <Prefix>   <Result> kB
 * and returns the integer result. */
static bool ReadMeminfoLine(FILE *MemInfo, const char *Prefix, int *Result)
{
    const size_t PrefixLength = strlen(Prefix);
    char Line[1024];
    while (fgets(Line, sizeof(Line), MemInfo))
    {
        if (strncmp(Line, Prefix, PrefixLength) == 0)
        {
            /* Good: this is our line. */
            if (sscanf(Line + PrefixLength, " %d ", Result) == 1)
                return true;
            else
            {
                printf("Malformed /proc/meminfo line:\n\t\"%s\"\n", Line);
                return false;
            }
        }
    }
    /* Oops.  Couldn't find anything. */
    printf("Unable to find \"%s\" line in /proc/meminfo\n", Prefix);
    return false;
}


/* Free memory processing is a little tricky.  By reading /proc/meminfo we
 * can discover "free" and "cached" memory, but turning this into a true free
 * memory number is more difficult.
 *    In general, the cached memory is effectively free ... but
 * unfortunately, files in the RAM file system also appear as "cached" and
 * are NOT free.  Even more unfortunately, it appears to be particularly
 * fiddly to determine how much space is used by the RAM file system! */
static void ProcessFreeMemory(void)
{
    FILE * MemInfo;
    if (TEST_NULL(MemInfo = fopen("/proc/meminfo", "r")))
    {
        int Free, Cached;   // In kB
        if (ReadMeminfoLine(MemInfo, "MemFree:", &Free)  &&
            ReadMeminfoLine(MemInfo, "Cached:",  &Cached))
        {
            RamfsUsage = (double) FindRamfsUsage() / SCALE_M;
            MemoryFree = (double) (Free + Cached) / SCALE_K - RamfsUsage;
        }
        fclose(MemInfo);
    }
}


/* The following reads the key system health parameters directly from the
 * appropriate devices and proc/sys files. */
static void ReadHealth(void)
{
    /* In case any of our reads fail, start by setting everything to zero,
     * which is generally an alarm condition. */
    memset(FanSpeeds, 0, sizeof(FanSpeeds));
    MbTemperature = 0;

    parse_file(proc_temp_mb,  1, "%d", &MbTemperature);
    MbTemperature /= 1000;
    parse_file(proc_fan0,     1, "%d", &FanSpeeds[0]);
    parse_file(proc_fan1,     1, "%d", &FanSpeeds[1]);
}



/*****************************************************************************/
/*                                                                           */
/*                            Network Monitoring                             */
/*                                                                           */
/*****************************************************************************/

static unsigned int CasChannelCount;     // Number of connected PVs
static unsigned int CasClientCount;      //  connected channel access clients

static uint32_t NetBytesRxLast, NetPacketsRxLast, NetMultiRxLast;
static uint32_t NetBytesTxLast, NetPacketsTxLast, NetMultiTxLast;

/* The following are the statistics we report, scaled to k bytes per second for
 * NetBytes{Rx,Tx} and packets per second for Net{Packets,Multi}{Rx,Tx}. */
static double NetBytesRxDelta, NetPacketsRxDelta, NetMultiRxDelta;
static double NetBytesTxDelta, NetPacketsTxDelta, NetMultiTxDelta;


static bool ReadNetworkStats(
    uint32_t *NetBytesRx, uint32_t *NetPacketsRx, uint32_t *NetMultiRx,
    uint32_t *NetBytesTx, uint32_t *NetPacketsTx, uint32_t *NetMultiTx)
{
    FILE *input;
    bool ok = TEST_NULL(input = fopen("/proc/net/dev", "r"));
    if (ok)
    {
        char line[1024];
        while (fgets(line, sizeof(line), input))
        {
            if (strncmp(line, "  eth0", 6) == 0)
            {
                ok = TEST_OK(sscanf(line,
                    "  eth0:%u %u %*d %*d %*d %*d %*d %u "
                           "%u %u %*d %*d %*d %*d %*d %u",
                    NetBytesRx, NetPacketsRx, NetMultiRx,
                    NetBytesTx, NetPacketsTx, NetMultiTx) == 6);
                break;
            }
        }
        fclose(input);
    }
    return ok;
}

static void ProcessNetworkStats(void)
{
    /* Channel access statistics. */
    casStatsFetch(&CasChannelCount, &CasClientCount);

    uint32_t NetBytesRx, NetPacketsRx, NetMultiRx;
    uint32_t NetBytesTx, NetPacketsTx, NetMultiTx;
    if (ReadNetworkStats(
            &NetBytesRx, &NetPacketsRx, &NetMultiRx,
            &NetBytesTx, &NetPacketsTx, &NetMultiTx))
    {
#define UPDATE(name, scale) \
    name##Delta = \
        ((double) scale / SENSORS_POLL_INTERVAL) * (name - name##Last); \
    name##Last = name
        UPDATE(NetBytesRx,   1e-3);
        UPDATE(NetMultiRx,   1);
        UPDATE(NetPacketsRx, 1);
        UPDATE(NetBytesTx,   1e-3);
        UPDATE(NetMultiTx,   1);
        UPDATE(NetPacketsTx, 1);
#undef UPDATE
    }
}


static void PublishNetworkStats(void)
{
    PUBLISH_READ_VAR(ulongin, "SE:CAPVS",  CasChannelCount);
    PUBLISH_READ_VAR(ulongin, "SE:CACLNT", CasClientCount);

    /* Read initial values to avoid crazy rates for first 10 seconds! */
    ReadNetworkStats(
        &NetBytesRxLast, &NetPacketsRxLast, &NetMultiRxLast,
        &NetBytesTxLast, &NetPacketsTxLast, &NetMultiTxLast);
    PUBLISH_READ_VAR(ai, "SE:NWBRX", NetBytesRxDelta);
    PUBLISH_READ_VAR(ai, "SE:NWPRX", NetPacketsRxDelta);
    PUBLISH_READ_VAR(ai, "SE:NWMRX", NetMultiRxDelta);
    PUBLISH_READ_VAR(ai, "SE:NWBTX", NetBytesTxDelta);
    PUBLISH_READ_VAR(ai, "SE:NWPTX", NetPacketsTxDelta);
    PUBLISH_READ_VAR(ai, "SE:NWMTX", NetMultiTxDelta);
}



/*****************************************************************************/
/*                                                                           */
/*                           NTP Status Monitoring                           */
/*                                                                           */
/*****************************************************************************/


enum {
    NTP_NOT_MONITORED,  // Monitoring disabled (or not yet happened)
    NTP_NO_NTP,         // No NTP server running locally
    NTP_STARTUP,        // Startup grace period
    NTP_NO_SYNC,        // NTP running but not synchronised
    NTP_OK,             // NTP running ok.
};
static unsigned int NTP_status = NTP_NOT_MONITORED;
static int NTP_stratum = 16;    // 16 means unreachable/invalid server
static EPICS_STRING NTP_server;


/* The NTP server can take more than 20 minutes to satisfy itself before
 * reporting synchronisation.  During this startup period we don't report an
 * error if synchronisation has not been established. */
static const int NTP_startup_window = 1500;


/* NTP/SNTP message packet (except for NTP control messages).  See RFC 1305
 * for NTP and RFC 2030 for SNTP.
 *    Note that as this packet goes over the wire, it is necessary to use
 * hton or ntoh transformations on all the fields. */

struct ntp_pkt {
    /* Bits 0-2: "mode" or message type:
     *           3 => client request, 4 => server response
     *           6 => NTP control message (different packet format)
     * Bits 3-5: NTP version number (can use 3 or 4 here)
     * Bits 6-7: Leap indicator and alarm indication (3 => unsynchronised). */
    uint8_t li_vn_mode;     // LI[7-6]:VN[5-3]:MODE[2-0]
    uint8_t stratum;        // Stratum level of clock
    int8_t ppoll;           // log_2(poll interval) in seconds
    int8_t precision;       // log_2(clock precision) in seconds
    int32_t rootdelay;      // 2^16 * Delay to reference in seconds
    int32_t rootdispersion; // 2^16 * Root dispersion in seconds
    int32_t refid;          // IP address of reference source (stratum > 1)
    uint64_t reftime;       // Time clock was last set (2^32*seconds in epoch)
    uint64_t org;           // Time this response left server
    uint64_t rec;           // Time this request received by server
    uint64_t xmt;           // Time this request left the client
};


/* This routine sends a single UDP message to the specified address and port,
 * and waits until timeout for a reply.  If a reply of some sort was received
 * this is returned with success.  Normal failure to receive a reply is
 * silent, as this is operationally normal and reported elsewhere.
 *     The timeout is in milliseconds, and cannot be more than 999. */
static bool UdpExchange(
    const char *address, in_port_t port, time_t timeout_ms,
    const void *tx_buffer, size_t tx_length,
    void *rx_buffer, size_t *rx_length)
{
    struct sockaddr_in ntp_server;
    memset(&ntp_server, 0, sizeof(ntp_server));
    ntp_server.sin_family = AF_INET;
    ntp_server.sin_port = (in_port_t) htons(port);  // bug in htons!
    inet_aton(address, &ntp_server.sin_addr);

    int sock;
    ssize_t rx = 0;
    bool Ok = TEST_IO(sock = socket(AF_INET, SOCK_DGRAM, 0));
    if (Ok)
    {
        ssize_t sent;

        fd_set rx_ready;
        FD_ZERO(&rx_ready);
        FD_SET(sock, &rx_ready);
        struct timeval timeout = { .tv_sec = 0, .tv_usec = 1000 * timeout_ms };

        int sel;
        Ok =
            TEST_IO(connect(sock,
                (const struct sockaddr *) &ntp_server, sizeof(ntp_server)))  &&
            TEST_IO(sent = send(sock, tx_buffer, tx_length, 0))  &&
            TEST_OK((size_t) sent == tx_length)  &&
            TEST_IO(sel = select(sock+1, &rx_ready, NULL, NULL, &timeout))  &&
            /* Fail if select timed out. */
            sel > 0  &&
            /* Read can fail, and we don't actually want to log this. */
            DO(rx = recv(sock, rx_buffer, *rx_length, 0))  &&
            rx != -1;

        IGNORE(TEST_IO(close(sock)));
    }

    *rx_length = Ok ? (size_t) rx : 0;
    return Ok;
}


/* Simply sends an SNTP packet to the given server, waits for a response or
 * timeout, and does simple validation of the response. */
static bool SNTP_exchange(
    const char *address, time_t timeout_ms, struct ntp_pkt *result)
{
    /* Might as well use the result packet for our transmit.  For a simple
     * SNTP status request we can just set the whole packet to zero (except
     * for the mode byte). */
    memset(result, 0, sizeof(struct ntp_pkt));
    result->li_vn_mode = (0 << 6) | (3 << 3) | (3 << 0);
    size_t rx = sizeof(struct ntp_pkt);
    return
        UdpExchange(address, 123, timeout_ms, result, rx, result, &rx)  &&
        /* Simple validation. */
        rx == sizeof(struct ntp_pkt)  &&       // Complete packet received
        (result->li_vn_mode & 7) == 4;  // Response is server mode response
}


/* For high stratum values the refid is the IP address of the reference
 * server, for stratum values 0 and 1 the refid is a four character string. */
static void refid_to_string(int stratum, int refid, EPICS_STRING *string)
{
#define ID_BYTE(i) ((refid >> (8*(i))) & 0xFF)
    if (stratum > 1)
        snprintf(string->s, sizeof(EPICS_STRING),
            "%d.%d.%d.%d", ID_BYTE(0), ID_BYTE(1), ID_BYTE(2), ID_BYTE(3));
    else
    {
        memcpy(string->s, &refid, sizeof(refid));
        string->s[4] = 0;
    }
#undef ID_BYTE
}


static void ProcessNtpHealth(void)
{
    struct ntp_pkt pkt;
    if (SNTP_exchange("127.0.0.1", 100, &pkt))
    {
        int LI = (pkt.li_vn_mode >> 6) & 3;
        NTP_status = LI == 3 ?
            (unsigned int) (LastUptime < NTP_startup_window ?
                NTP_STARTUP : NTP_NO_SYNC) : NTP_OK;
        NTP_stratum = pkt.stratum == 0 ? 16 : pkt.stratum;
        refid_to_string(pkt.stratum, pkt.refid, &NTP_server);
    }
    else
    {
        NTP_status = NTP_NO_NTP;
        NTP_stratum = 16;
        strcpy(NTP_server.s, "no server");
    }
}




/*****************************************************************************/
/*                                                                           */
/*                               Fan Control                                 */
/*                                                                           */
/*****************************************************************************/

/* This code makes a half hearted attempt to regulate the temperature by
 * controlling the fans.  Unfortunately we have very tight limits on the speeds
 * we're able to select. */

/* Limits on controlled fan speeds: the controller won't attempt to push beyond
 * these limits. */
#define MAX_FAN_SPEED   5700
/* This minimum speed of 4,300 RPM is specified by Instrumentation Technologies.
 * If the fans are driven at lower speeds then their drive transistors can be
 * overloaded!  This constraint was reported in an e-mail from Matjaz Znidarcic
 * dated 1st July 2009.  A minimum set speed of 4,100 appears to be acceptable,
 * according to i-Tech. */
#define MIN_FAN_SPEED   4100

/* It really doesn't matter hugely how we start, so to simplify things we assume
 * an initial fan speed of 4,500 RPM.  The controller will settle quickly enough
 * anyhow.
 *    The one disadvantage of not reading the fan speed at startup is that
 * restarting the health daemon will force the controller to hunt for the right
 * speed again.  Not a big deal. */
#define INITIAL_FAN_SPEED   4500

/* Default controller parameters when using motherboard sensor. */
#define TARGET_TEMP_MB          42
#define CONTROLLER_KP_MB        40
#define CONTROLLER_KI_MB        40


static int target_temperature = TARGET_TEMP_MB;
static int fan_control_integral = 0;
static int controller_KP = CONTROLLER_KP_MB;
static int controller_KI = CONTROLLER_KI_MB;
static int fan_set_speed;


static void write_device(const char *device, int value)
{
    FILE *output = fopen(device, "w");
    if (TEST_NULL(output))
    {
        fprintf(output, "%d", value);
        fclose(output);
    }
}


/* We run a very simple PI control loop, setting the fan speeds to regulate
 * the selected temperature sensor. */
static void step_fan_control(void)
{
    int error = MbTemperature - target_temperature;
    fan_control_integral += error;
    fan_set_speed = INITIAL_FAN_SPEED +
        error * controller_KP + fan_control_integral * controller_KI;

    /* Prevent integrator windup when speed reaches its limits. */
    if (fan_set_speed > MAX_FAN_SPEED)
    {
        fan_set_speed = MAX_FAN_SPEED;
        fan_control_integral -= error;
    }
    else if (fan_set_speed < MIN_FAN_SPEED)
    {
        fan_set_speed = MIN_FAN_SPEED;
        fan_control_integral -= error;
    }

    /* Write the new target fan speed. */
    write_device(proc_fan0_set, fan_set_speed);
    write_device(proc_fan1_set, fan_set_speed);
}


static void initialise_fan_control(void)
{
    PUBLISH_READ_VAR(longin, "SE:FAN_SET", fan_set_speed);
    PUBLISH_WRITE_VAR_P(longout, "SE:TEMP", target_temperature);

    /* Initialise fan speed control. */
    write_device(proc_fan0_pwm, 2);
    write_device(proc_fan1_pwm, 2);
}



/*****************************************************************************/
/*                                                                           */
/*                     Sensors Initialisation and Control                     */
/*                                                                           */
/*****************************************************************************/

static struct epics_interlock *interlock;


static void ProcessSensors(void)
{
    ProcessUptimeAndIdle();
    ProcessFreeMemory();
    ProcessNetworkStats();

    ReadHealth();
    step_fan_control();

    ProcessNtpHealth();
}


static void *sensors_thread(void *context)
{
    while (true)
    {
        interlock_wait(interlock);
        ProcessSensors();
        interlock_signal(interlock, NULL);

        sleep(SENSORS_POLL_INTERVAL);
    }
    return NULL;
}


static bool overflows[PULSED_BIT_COUNT];

static void read_overflows(void)
{
    const bool read_mask[PULSED_BIT_COUNT] = {
        [OVERFLOW_ADC_LIMIT] = true,
        [OVERFLOW_ADC_FILTER] = true,
        [OVERFLOW_FIR] = true,
        [OVERFLOW_DAC] = true,
        [OVERFLOW_DAC_COMP] = true,
        [OVERFLOW_FIR_DECIMATE] = true,
    };
    hw_read_pulsed_bits(read_mask, overflows);
}


static bool read_clock_dropout(void)
{
    bool dropout = hw_read_clock_dropout();
    if (dropout)
        print_error("ADC clock dropout detected");
    return dropout;
}


bool initialise_sensors(void)
{
    interlock = create_interlock("SE", false);

    PUBLISH_READ_VAR(longin, "SE:TEMP", MbTemperature);
    PUBLISH_READ_VAR(longin, "SE:FAN1", FanSpeeds[0]);
    PUBLISH_READ_VAR(longin, "SE:FAN2", FanSpeeds[1]);

    PUBLISH_READ_VAR(ai, "SE:FREE",    MemoryFree);
    PUBLISH_READ_VAR(ai, "SE:RAMFS",   RamfsUsage);
    PUBLISH_READ_VAR(ai, "SE:UPTIME",  UptimeHours);
    PUBLISH_READ_VAR(ai, "SE:EPICSUP", EpicsUpHours);
    PUBLISH_READ_VAR(ai, "SE:CPU",     CpuUsage);

    PUBLISH_READER(bi, "SE:ADCCLK", read_clock_dropout);

    PublishNetworkStats();

    PUBLISH_READ_VAR(mbbi,     "SE:NTPSTAT", NTP_status);
    PUBLISH_READ_VAR(longin,   "SE:STRATUM", NTP_stratum);
    PUBLISH_READ_VAR(stringin, "SE:SERVER",  NTP_server);

    PUBLISH_ACTION("SE:OVF:SCAN", read_overflows);
    PUBLISH_READ_VAR(bi, "SE:OVF:ADCIN", overflows[OVERFLOW_ADC_LIMIT]);
    PUBLISH_READ_VAR(bi, "SE:OVF:ADCCOMP", overflows[OVERFLOW_ADC_FILTER]);
    PUBLISH_READ_VAR(bi, "SE:OVF:FIR", overflows[OVERFLOW_FIR]);
    PUBLISH_READ_VAR(bi, "SE:OVF:DAC", overflows[OVERFLOW_DAC]);
    PUBLISH_READ_VAR(bi, "SE:OVF:COMP", overflows[OVERFLOW_DAC_COMP]);
    PUBLISH_READ_VAR(bi, "SE:OVF:DECIMATE", overflows[OVERFLOW_FIR_DECIMATE]);

    InitialiseUptime();
    initialise_fan_control();

    pthread_t thread_id;
    return
        InitialiseRamfsUsage()  &&
        TEST_PTHREAD(pthread_create(&thread_id, NULL, sensors_thread, NULL));
}
