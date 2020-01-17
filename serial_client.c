#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#ifdef RPI
#include <wiringPi.h>
#define WRITE_SIG 27
#define READ_SIG 17
#endif

int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("error from tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0;        // no signaling chars, no echo,
                            // no canonical processing
    tty.c_oflag = 0;        // no remapping, no delays
    tty.c_cc[VMIN] = 0;     // read doesn't block
    tty.c_cc[VTIME] = 5;    // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                       // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS; // gnu99 compilation

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
         perror("error from tcsetattr");
        return -1;
    }
    return 0;
}

void set_blocking(int fd, int should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("error from tggetattr");
        return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
         perror("error setting term attributes");
}

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv)
{
    #ifdef RPI
    if (wiringPiSetup()==-1)
    {
        printf("Could not setup signalling using wiringPi. Use sudo..?\n") ;
        pinMode(READ_SIG, OUTPUT);
        pinMode(WRITE_SIG, OUTPUT);
    }
    #endif
    srand(time(NULL)) ;
    if (argc != 3)
    {
        printf("Usage: ./serial_client <PORT> <BAUD>");
        exit(0);
    }
    int fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("error %d opening %s: %s", errno, argv[1], strerror(errno));
        return 1;
    }
    int baud = atol(argv[2]);
    unsigned charsleep = 0 ;
    switch (baud)
    {
    case 115200:
        set_interface_attribs(fd, B115200, 0);
        charsleep = 100 ;
        break;

    case 9600:
        set_interface_attribs(fd, B9600, 0);
        charsleep = 1000 ;
        break;

    case 230400:
        set_interface_attribs(fd, B230400, 0);
        charsleep = 50 ;
        break;
    default:
        set_interface_attribs(fd, B115200, 0);
        charsleep = 100 ;
        break;
    }
    set_blocking (fd, 0);

    unsigned char inbuf[34], obuf ;
    while(1)
    {
        #ifdef RPI
        digitalWrite(READ_SIG,1) ;
        #endif
        usleep(34*charsleep) ;
        int n = read (fd, inbuf, 34) ;
        #ifdef RPI
        digitalWrite(READ_SIG,0) ;
        #endif
        obuf = 0xff & rand() ;
        #ifdef RPI
        digitalWrite(WRITE_SIG,1) ;
        #endif
        n = write(fd, &obuf , 1) ;
        #ifdef RPI
        digitalWrite(WRITE_SIG,0) ;
        #endif
        usleep(charsleep) ;
        printf("Received: ") ;
        for ( int i = 0 ; i < 34 ; i++ )
            printf("%d ",  inbuf[i] ) ;
        printf("\nSent: %d\n", obuf) ;
    }
    return 0 ;
}