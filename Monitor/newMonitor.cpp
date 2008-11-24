/*
 * Monitor.c: Simple program to read/write from/to any location in memory.
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

#define ASSERT(e) if(!(e)) AssertFail(__FILE__, __LINE__)

void AssertFail(const char * FileName, int LineNumber)
{
    fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n",
            LineNumber, FileName, errno, strerror(errno));
    exit(1);
}
  
 
//#define MAP_SIZE 4096UL
#define MAP_SIZE (1 << 14)
#define MAP_MASK (MAP_SIZE - 1)


bool Write = false;
bool Binary = false;

void Usage(FILE * file)
{
    fprintf(file,
"Usage: monitor [-w] [-b] start length\n"
"    -w    Read from stdin and write into memory\n"
"    -b    Read or write in binary mode\n");
}

void ReadArguments(int argc, char **argv, int &Start, int &Length)
{
    /* First read the options. */
    for (bool reading = true; reading; )
    {
        switch (getopt(argc, argv, "+hbw"))
        {
            case 'w':
                Write = true;
                break;
            case 'b':
                Binary = true;
                break;
            case '?':
            default:
                fprintf(stderr, "Try `%s -h` for usage\n", argv[0]);
                exit(1);
            case 'h':
                Usage(stdout);
                exit(0);
            case -1:
                reading = false;
                break;
        }
    }

    /* Now decode start and length. */
    argc -= optind;
    argv += optind;
    if (argc != 2)
    {
        Usage(stderr);
        exit(1);
    }
    else
    {
        Start = strtoul(argv[0], NULL, 0);
        Length = strtoul(argv[1], NULL, 0);
    }
}


void ReadMemory(int *Start, int Length)
{
    for (int i = 0; i < Length; i ++)
    {
        printf("%08x ", Start[i]);
        if (i % 8 == 7)
            printf("\n");
    }
    if (Length % 8 != 0)
        printf("\n");
}


bool ReadIntFromStdin(int &Read)
{
    return fscanf(stdin, " %x", &Read) == 1;
}


void WriteMemory(int *Start, int Length)
{
    int Read;
    for (int i = 0; i < Length  &&  ReadIntFromStdin(Read); i ++)
        Start[i] = Read;
}


int main(int argc, char **argv)
{
    int Start, Length;
    ReadArguments(argc, argv, Start, Length);

    int mem = open("/dev/mem", O_RDWR | O_SYNC);
    ASSERT(mem != -1);
//    fflush(stdout);
    
    /* Map one page */
    char * map_base = (char *) mmap(
        0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
        mem, Start & ~MAP_MASK);
    ASSERT(map_base != (void *) -1);
    int *start_addr = (int *) (map_base + (Start & MAP_MASK));

    if (Write)
        WriteMemory(start_addr, Length);
    else
        ReadMemory(start_addr, Length);

    ASSERT(munmap(map_base, MAP_SIZE) != -1);
    close(mem);
    return 0;
#if 0
    
    
    int fd;
    char *map_base, *virt_addr; 
    unsigned long read_result, writeval;
    off_t target;
    int access_type = 'w';
	
    if(argc < 2)
    {
        fprintf(stderr,
            "\nUsage:\t%s { address } [ type [ data ] ]\n"
            "\taddress : memory address to act upon\n"
            "\ttype    : access operation type : [b]yte, [h]alfword, [w]ord\n"
            "\tdata    : data to be written\n\n", argv[0]);
        exit(1);
    }
    
    target = strtoul(argv[1], 0, 0);
    if(argc > 2)
        access_type = tolower(argv[2][0]);

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    ASSERT(fd != -1);
    fflush(stdout);
    
    /* Map one page */
    map_base = (char *) mmap(
        0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
        fd, target & ~MAP_MASK);
    ASSERT(map_base != (void *) -1);
    
    virt_addr = map_base + (target & MAP_MASK);
    if(argc <= 3)
    {
        switch(access_type)
        {
            case 'b':
                read_result = *((unsigned char *) virt_addr);
                break;
            case 'h':
                read_result = *((unsigned short *) virt_addr);
                break;
            case 'w':
                read_result = *((unsigned long *) virt_addr);
                break;
            default:
                fprintf(stderr, "Illegal data type '%c'.\n", access_type);
                exit(2);
	}
	
        printf("Value at address 0x%X (%p): 0x%X\n",
               target, virt_addr, read_result); 
        fflush(stdout);
    }

    if(argc > 3)
    {
        writeval = strtoul(argv[3], 0, 0);
        switch(access_type)
        {
            case 'b':
                *((unsigned char *) virt_addr) = writeval;
                break;
            case 'h':
                *((unsigned short *) virt_addr) = writeval;
                break;
            case 'w':
                *((unsigned long *) virt_addr) = writeval;
                break;
        }
        fflush(stdout);
    }
	
    ASSERT(munmap(map_base, MAP_SIZE) != -1);
    close(fd);
    return 0;
#endif
}
