// test-com.c
#define 	DEBUGSS	0

#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>

#define true		1
#define false		0

void init_keyboard();
void close_keyboard();
int kbhit();
int readch();

int
SetComAttr(int fd)
{
    int			n;
    struct termios	term;

  // ボーレート等を設定
    n = tcgetattr(fd, &term);
    if (n < 0)
	return n;

    bzero(&term, sizeof(term));

    term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
    term.c_iflag = IGNPAR;
    term.c_oflag = 0;
    term.c_lflag = 0;/*ICANON;*/

    term.c_cc[VINTR]    = 0;     /* Ctrl-c */
    term.c_cc[VQUIT]    = 0;     /* Ctrl-? */
    term.c_cc[VERASE]   = 0;     /* del */
    term.c_cc[VKILL]    = 0;     /* @ */
    term.c_cc[VEOF]     = 4;     /* Ctrl-d */
    term.c_cc[VTIME]    = 0;
    term.c_cc[VMIN]     = 0;
    term.c_cc[VSWTC]    = 0;     /* '?0' */
    term.c_cc[VSTART]   = 0;     /* Ctrl-q */
    term.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    term.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    term.c_cc[VEOL]     = 0;     /* '?0' */
    term.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    term.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    term.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    term.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    term.c_cc[VEOL2]    = 0;     /* '?0' */

  //	tcflush(fd, TCIFLUSH);
    n = tcsetattr(fd, TCSANOW, &term);

    return n;
}

int
main()
{
    int			status;
    FILE		*file = NULL;
    int			fd = -1;
    char		fname[64];
    char		devname[64];
    char		str[256];
    unsigned short	data[6];
    int			comNo;
    int			tick;
    int			clk, clkb, clkb2, clk0;
    int			tw;
    int			num;
    int			n;

  start:
  // COMポートをオープン
    printf("Enter COM port > ");
    scanf("%d", &comNo);
    printf("Open /dev/ttyUSB%d\n", comNo);

    sprintf(devname, "/dev/ttyUSB%d", comNo);
    fd = open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
	goto over;

  // サンプリング周期を得る
    tw = 16;
    printf("Enter sampling time (ms) > ");
    scanf("%d", &tw);
    printf("Sampling time = %d ms\n", tw);

    printf("Enter File name > ");
    scanf("%s", fname);
    file = fopen(fname, "w");
    if (!file)
	goto over;

  // COMポートのボーレート等を設定
    if (SetComAttr(fd) < 0)
    {
	printf("Failed to set termios paramters");
	goto over;
    }

  // データを読み出す
    printf("=== record data ===\n");
    clk0 = clock() / (CLOCKS_PER_SEC / 1000);
    clkb = 0;
    clkb2 = 0;
    num = 0;

  // 単データリクエスト（初回分）
    write(fd, "R", 1);

    init_keyboard();

    while (true)
    {
      // サンプリング周期だけ待つ
	while (true)
	{
	    clk = clock() / (CLOCKS_PER_SEC / 1000) - clk0;

	    if (clk >= clkb + tw)
	    {
		clkb = clk / tw * tw;
		break;
	    }
	}

      // 単データリクエスト（次回分）
	write(fd, "R", 1);

      // 単データを得る
	n = read(fd, str, 27);
	if (n < 27)
	{
	  //			printf ("=== error ! n = %d ===\n", n);
	    goto skip;
	}

	sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx", &tick,
	       &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

	sprintf(str, "%05d,%d,%05d,%05d,%05d,%05d,%05d,%05d\n",
		clk / tw * tw, tick,
		data[0], data[1], data[2], data[3], data[4], data[5]);

	fputs(str, file);
	num++;

      skip:
	if (clk >= 10000)
	    break;

      // コンソールに間引き表示
	if (clk >= clkb2 + 1000)
	{
	    puts(str);
	    if (kbhit() && readch() == '.')
		break;
	    clkb2 = clk / 1000 * 1000;
	}
    }

  over1:
    close_keyboard();
  over:
    if (file)
    {
	fclose(file);
	file = NULL;
    }

    if (fd >= 0)
    {
	close(fd);
	fd = -1;
    }

    printf ("=== num = %d ===\n", num);

    printf("exit (y / n) ? > ");
    scanf("%s", str);
    if (str[0] == 'y')
    {
      //		exit(0);
    }
    else
    {
	goto start;
    }
}
