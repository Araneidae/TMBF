/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2007  Michael Abbott, Diamond Light Source Ltd.
 *
 * The Libera EPICS Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * The Libera EPICS Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:
 *      Dr. Michael Abbott,
 *      Diamond Light Source Ltd,
 *      Diamond House,
 *      Chilton,
 *      Didcot,
 *      Oxfordshire,
 *      OX11 0DE
 *      michael.abbott@diamond.ac.uk
 */


#include <errno.h>

void print_error(const char * Message, const char * FileName, int LineNumber);

/* Helper macros for OS calls: in all cases the call is wrapped by a macro
 * which converts the return code into a boolean value.  If the call fails
 * then an automatically generated error message (including filename and line
 * number) is printed.
 *     In all cases true is returned iff the call is successful.
 *
 * There are three main cases, depending on how the return value is
 * interpreted:
 *
 *  TEST_IO(Result, function, arguments)
 *      Computes
 *          Result = function(arguments)
 *      and reports an error message if Result == -1.
 *  
 *  TEST_(function, arguments)
 *      Computes
 *          function(arguments)
 *      and reports an error message if the function returns -1.
 *  
 *  TEST_0(function, arguments)
 *      Computes
 *          function(arguments)
 *      and reports an error message if the function returns any non-zero
 *      value.  This is designed for use with the pthread_ family of
 *      functions, and the returned value is assigned to errno.
 *
 * The following is slightly different in purpose.
 *  
 *  TEST_OK(test)
 *      Evaluates test as a boolean and prints an error message if it
 *      evaluates to false. */

#define TEST_IO(var, command, args...) \
    ( { \
        var = (command)(args); \
        if ((int) var == -1) \
            print_error(#var " = " #command "(" #args ")", \
                __FILE__, __LINE__); \
        (int) var != -1; \
    } )

#define TEST_(command, args...) \
    ( { \
        bool __ok__ = (command)(args) != -1; \
        if (!__ok__) \
            print_error(#command "(" #args ")", __FILE__, __LINE__); \
        __ok__; \
    } )

#define TEST_0(command, args...) \
    ( { \
        errno = (command)(args); \
        if (errno != 0) \
            print_error(#command "(" #args ")", __FILE__, __LINE__); \
        errno == 0; \
    } )

#define TEST_OK(test) \
    ( { \
        bool __ok__ = (test); \
        if (!__ok__) \
            { errno = 0; print_error(#test, __FILE__, __LINE__); } \
        __ok__; \
    } )


/* These two macros facilitates using the macros above by creating an if
 * expression that's slightly more sensible looking than ?: in context. */
#define DO_(action)                     ({action; true;})
#define IF_(test, iftrue)               ((test) ? (iftrue) : true)
#define IF_ELSE(text, iftrue, iffalse)  ((test) ? (iftrue) : (iffalse))



/* A rather randomly placed helper routine.  This and its equivalents are
 * defined all over the place, but there doesn't appear to be a definitive
 * definition anywhere. */
#define ARRAY_SIZE(a)   (sizeof(a)/sizeof((a)[0]))


/* Here is something cute and evil from
 *  http://www.jaggersoft.com/pubs/CVu11_3.html
 * Compile time assertion tests! */
#define COMPILE_TIME_ASSERT(pred) \
    do switch(0) { case 0: case pred:; } while (0)
//#define COMPILE_TIME_ASSERT(pred) 

