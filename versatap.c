/* -*- Mode: C; tab-width: 4 -*- */
#include <stdlib.h>
#include <Stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main(int argc, char **argv)
{
    int fd;
    fd = open('/dev/ttyUSB2', O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Could not open USB to Serial convereter device");
        exit(1);
    }

    
  
