/** Minimal program for ddr2 ram initialization. */

/*
Libera GNU/Linux DDR2 Initialization
Copyright (C) 2004-2006 Instrumentation Technologies

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
or visit http://www.gnu.org
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>


// range of BASE RAM
#define     NEUTRINO_BASE               0x14000000
#define     NEUTRINO_END                0x1403BFFF
#define     NEUTRINO_SIZE               (NEUTRINO_END - NEUTRINO_BASE)

// DDR INIT commands
#define     NOP                         0x47000000
#define     PRECHARGE                   0x42000400
#define     REFRESH                     0x41000000
#define     MRS                         0x40000443
#define     EMRS1                       0x40010014
#define     EMRS2                       0x40020000
#define     EMRS3                       0x40030000


//-----------------------------------------------------------------------------
static void *cep_init(int *fd_out, off_t mm_offset, off_t mm_size)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(fd == -1)
    {
        fprintf(stdout, "can not open /dev/mem\n");
        exit(-1);
    }

    void *map_base = mmap(
        0, mm_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
        mm_offset & ~(getpagesize()-1));
    if (map_base == ((void *) -1))
    {
        fprintf(stdout, "problem mapping /dev/mem\n");
        exit(-2);
    }

    *fd_out = fd;
    return map_base;
}

//-----------------------------------------------------------------------------
// read & write function
//-----------------------------------------------------------------------------

static inline unsigned long read_w(void *base, off_t offset)
{
    return *((volatile unsigned long *) (base+offset));
}

static inline void write_w(void *base, off_t offset, unsigned long value)
{
    *((volatile unsigned long *) (base+offset)) = value;
}

//-----------------------------------------------------------------------------
// DDR initialization
//-----------------------------------------------------------------------------
static void ddr_init(void *base)
{
    // disable SDRAM
    write_w(base, 0x1801C, 0x0);
    // SDRAM configuration sequence
    write_w(base, 0x18018, NOP);    // NOP

    sleep(1);
    write_w(base, 0x18018, PRECHARGE);  // precharge all command

    sleep(1);
    write_w(base, 0x18018, EMRS2);  // EMRS2 command
    write_w(base, 0x18018, EMRS3);  // EMRS3 command
    write_w(base, 0x18018, EMRS1);  // EMRS DLL enable
    write_w(base, 0x18018, MRS | (0x1 <<  8));
    // DLL reset, A8 high, BA0 low

    write_w(base, 0x18018, PRECHARGE);  // precharge all command
    write_w(base, 0x18018, REFRESH);    // refresh command
    write_w(base, 0x18018, REFRESH);    // refresh command
    write_w(base, 0x18018, REFRESH);    // refresh command
    write_w(base, 0x18018, REFRESH);    // refresh command
    write_w(base, 0x18018, MRS);
    // initializing device operation (program operating parameters)
    sleep(1);

    write_w(base, 0x18018, EMRS1 | (0x1 << 9) | (0x1 << 8) | (0x1 << 7));
    // OCD calibration (set to default value)
    write_w(base, 0x18018, EMRS1);  // OCD exit
    // enable SDRAM
    write_w(base, 0x1801C, 0x1);
}

//-----------------------------------------------------------------------------
// main function
//-----------------------------------------------------------------------------
int main(void)
{
    int fd;
    void *map_base = cep_init(&fd, NEUTRINO_BASE, NEUTRINO_SIZE);

    ddr_init(map_base);

    return 0;
}
