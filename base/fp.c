/* $Id: fp.c 2192 2008-10-07 09:13:06Z matejk $ */
/*
 fpgaLoad - program for downloading  the  FPGA Virtex  XCVP20 prom file
 in binary format  into the device.
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>


#define CPLD_ADD 0x10000000
#define REG2 0x04
#define REG3 0x06

#define FPGA_CS 0x80
#define FPGA_RW 0x40
#define PROG_B  0x20
#define MODE    0x10

#define DONE    0x80
#define INT_B   0x40
#define BUSY    0x20

#define REG2_MASK 0x0f
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

static volatile unsigned char *Reg2Address, *Reg3Address;
/*
  PROG_B  -----\______/-----------------------------------------------
  INIT_B  -------\__________/-----------------------------------------
  CCLK    -------------------------\____/----\____/----\____/----------
  FPGA_CS ----------------------\_______________________________/------
  FPGA_RW ----------\___________________________________________/------
  DATA    ===========================x=========x=========x=============
  BUSY    -------------------\__________________________________/------
  SEQUENCE |1  |2   |3|4        |5      |6        |7            |N+1
*/


// Generate a cclk signal with a dummy read
static inline void cclk(void)
{
    *Reg3Address = (unsigned char) 0;
}


int main(int argc, char **argv)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1)
    {
        fprintf(stderr, "can't open /dev/mem \n");
        exit(-1);
    }

    unsigned char *mapBase = mmap(
        0, MAP_SIZE, PROT_READ | PROT_WRITE,
        MAP_SHARED, fd, CPLD_ADD  & ~MAP_MASK);
    if (mapBase == MAP_FAILED)
    {
        fprintf(stderr, "can't mmap REG2 \n");
        exit(-1);
    }

    Reg2Address = mapBase + (REG2 & MAP_MASK);
    Reg3Address = mapBase + (REG3 & MAP_MASK);

    unsigned char Reg2 = *Reg2Address & REG2_MASK;
    Reg2 &= ~MODE;                          // Virtex is in configuration mode
    Reg2 |= PROG_B | FPGA_RW | FPGA_CS;     // sequence 1
    *Reg2Address = Reg2;
    usleep(1000);

    Reg2 &= ~PROG_B;                         // sequence 2
    *Reg2Address = Reg2;
    for (int i = 0; i < 9; i ++)
        cclk();
    usleep(1000);

    unsigned char check = *Reg2Address;
    if (check & INT_B)
    {
        fprintf(stderr, "INIT_B signal is not LOW reg2= 0x%02x\n", check);
        exit(-1);
    }

    Reg2 |=  PROG_B;                        // sequence 4
    *Reg2Address = Reg2;

    int init_count = 0;
    do {
        check = *Reg2Address;
        if (init_count++ > 5000)
        {
            printf("INIT_B signal is not HIGH reg2= 0x%02x\n", check);
            exit(-1);
        }
    } while (!(check & INT_B));

    Reg2 &= ~(FPGA_RW | FPGA_CS);                        // sequence 3
    *Reg2Address = Reg2;
    usleep(1000);

    /* Data */
    unsigned char StreamData[1024];
    size_t len;
    while (
        len = fread(StreamData, 1, sizeof(StreamData), stdin),
        len > 0)
    {
        for (size_t i = 0; i < len; i++)
            *Reg3Address = StreamData[i];
    }


    Reg2 |= FPGA_CS | FPGA_RW;           // sequence N+1
    *Reg2Address = Reg2;

    int done_count = 0;
    while (!(*Reg2Address & DONE))
    {
        if (done_count++ > 1000)
        {
            fprintf(stderr, " DONE signal is not HIGH Reg2=0x%02x\n",
                *Reg2Address);
            break;
        }
        cclk();
    }

    Reg2 |= MODE;              // Virtex is in regular operation mode
    *Reg2Address = Reg2;

    close(fd);
    return 0;
}
