/*
    Copyright (C) 2003-2010  Egon Wanke <blitzortung@gmx.org>

    Modified for Linux kernel 2.4 by Tim Jefford 2010 <timjefford@kilke.co.uk>

    Name this file to "tracker_Linux.c" and compile it by
    > g++ -Wall -lm -o tracker_Linux tracker_Linux.c

    then try
    > ./tracker_blitzortun -h
*/

// if you have problems with nonterminating child processes, uncomment the follogin line
#define KERNEL_2_4


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#define VERSION                 "LinuxTracker&nbsp;Ver.&nbsp;16.0" // version string send to server
#define SLEEP_TIME_SEC          20                                 // time interval for sending data
//#define STRIKES_SERVER_ADDR     "85.214.99.132"                    // server address
#define STRIKES_SERVER_ADDR     "rechenserver.de"                  // server address
#define STRIKE_SERVER_PORT      8308                               // server port
#define STRING_BUFFER_SIZE      512                                // maximal size for strings
#define AD_MAX_VALUE            128                                // maximal absolut value for channel A and B
#define AD_MAX_VOLTAGE          2500                               // maximal absolute millivolt for channel A and B
#define AD_THRESHOLD_VOLTAGE    500                                // absolute threshold millivolt for channel A and B
#define STRIKE_BUFFER_SIZE      10240                              // Buffer size for strikes
#define RING_BUFFER_SIZE        60                                 // ring buffer size for averaged computaion of lat, lon, alt
#define PRECISION               0.000001000                        // time presision to reach befor sende

// initialization for San Jose Navigation moduls, 4800 baud
char init_gps_sjn[]="\
$PFEC,GPint,GGA01,GLL00,GSA00,GSV00,RMC01,DTM00,VTG00,ZDA00*00\n";

// initialization for Garmin moduls, 4800 baud
char init_gps_garmin_4800[]= "\
$PGRMO,GPGGA,1*00\n\
$PGRMO,GPGSA,0*00\n\
$PGRMO,GPGSV,0*00\n\
$PGRMO,GPRMC,1*00\n\
$PGRMO,GPVTG,0*00\n\
$PGRMO,PGRMM,0*00\n\
$PGRMO,PGRMT,0*00\n\
$PGRMO,PGRME,0*00\n\
$PGRMO,PGRMB,0*00\n\
$PGRMCE*00\n\
$PGRMC,,51.5,,,,,,,,3,,2,4,*00\n";

// initialization for Garmin moduls, 9600 baud
char init_gps_garmin_9600[]= "\
$PGRMO,GPGGA,1*00\n\
$PGRMO,GPGSA,0*00\n\
$PGRMO,GPGSV,0*00\n\
$PGRMO,GPRMC,1*00\n\
$PGRMO,GPVTG,0*00\n\
$PGRMO,PGRMM,0*00\n\
$PGRMO,PGRMT,0*00\n\
$PGRMO,PGRME,0*00\n\
$PGRMO,PGRMB,0*00\n\
$PGRMCE*00\n\
$PGRMC,,51.5,,,,,,,,4,,2,4,*00\n";

// initialization for Garmin moduls, 19200 baud
char init_gps_garmin_19200[]= "\
$PGRMO,GPGGA,1*00\n\
$PGRMO,GPGSA,0*00\n\
$PGRMO,GPGSV,0*00\n\
$PGRMO,GPRMC,1*00\n\
$PGRMO,GPVTG,0*00\n\
$PGRMO,PGRMM,0*00\n\
$PGRMO,PGRMT,0*00\n\
$PGRMO,PGRME,0*00\n\
$PGRMO,PGRMB,0*00\n\
$PGRMCE*00\n\
$PGRMC,,51.5,,,,,,,,5,,2,4,*00\n";

// initialization for SiRF moduls, 4800 baud
char init_gps_sirf_4800[]= "\
$PSRF103,00,00,01,01*00\n\
$PSRF103,01,00,00,01*00\n\
$PSRF103,02,00,00,01*00\n\
$PSRF103,03,00,00,01*00\n\
$PSRF103,04,00,01,01*00\n\
$PSRF103,05,00,00,01*00\n\
$PSRF103,06,00,00,01*00\n\
$PSRF103,08,00,00,01*00\n\
$PSRF100,1,4800,8,1,0*00\n";

// initialization for SiRF moduls, 9600 baud
char init_gps_sirf_9600[]= "\
$PSRF103,00,00,01,01*00\n\
$PSRF103,01,00,00,01*00\n\
$PSRF103,02,00,00,01*00\n\
$PSRF103,03,00,00,01*00\n\
$PSRF103,04,00,01,01*00\n\
$PSRF103,05,00,00,01*00\n\
$PSRF103,06,00,00,01*00\n\
$PSRF103,08,00,00,01*00\n\
$PSRF100,1,9600,8,1,0*00\n";

// initialization for SiRF moduls, 19200 baud
char init_gps_sirf_19200[]= "\
$PSRF103,00,00,01,01*00\n\
$PSRF103,01,00,00,01*00\n\
$PSRF103,02,00,00,01*00\n\
$PSRF103,03,00,00,01*00\n\
$PSRF103,04,00,01,01*00\n\
$PSRF103,05,00,00,01*00\n\
$PSRF103,06,00,00,01*00\n\
$PSRF103,08,00,00,01*00\n\
$PSRF100,1,19200,8,1,0*00\n";

// initialization for SiRF moduls, 38400 baud
char init_gps_sirf_38400[]= "\
$PSRF103,00,00,01,01*00\n\
$PSRF103,01,00,00,01*00\n\
$PSRF103,02,00,00,01*00\n\
$PSRF103,03,00,00,01*00\n\
$PSRF103,04,00,01,01*00\n\
$PSRF103,05,00,00,01*00\n\
$PSRF103,06,00,00,01*00\n\
$PSRF103,08,00,00,01*00\n\
$PSRF100,1,38400,8,1,0*00\n";

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

extern int errno;
bool verbose_flag;
long long sleep_time;

struct strike_type {
  int year;
  int mon;
  int day;
  int hour;
  int min;
  int sec;
  long long nsec;
  long double lat;
  long double lon;
  long double alt;
  int A;
  int B;
  char status; };

struct strike_type strikes [STRIKE_BUFFER_SIZE];

int cnt1= 0;
int cnt2= 0;
int cnt3= 0;

char *device;
char *username;
char *password;

int last_year, last_mon, last_day, last_hour, last_min, last_sec;
long double last_lat, last_lon, last_alt;
long long last_time;
long long last_time_send= 0ll;
char last_status;

long long counter_ring_buffer [RING_BUFFER_SIZE];
struct position_type {
  long double lat;
  long double lon;
  long double alt; } position_ring_buffer [RING_BUFFER_SIZE];
int ring_buffer_cnt= 0;

long long counter_last;
long long counter_sum= 1;
long double counter_average;
long double counter_precision;

pid_t cld_pid= 0;

/******************************************************************************/
/***** initialization *********************************************************/
/******************************************************************************/

//
char int_to_hex (int b)
{
  b&= 0x0F;
  if (b > 9) {
    return(b+'A'-10); }
  else {
    return (b+'0'); }
}

// computes NMEA checksums between every $ and *
void fill_checksum (char *buf)
{
  unsigned int i;
  int c= 0;

  for (i=0; i < strlen (buf); i++) {
    if (buf[i] == '$') {
      c= 0; }
    else if (buf[i] == '*') {
      i++;
      buf [i]= int_to_hex (c>>4);
      i++;
      buf [i]= int_to_hex (c); }
    else {
      c^=buf[i]; } }
}

// initialize the GPS modul
void init_gps (int f, char *gps_device)
{
  char init_string [STRING_BUFFER_SIZE];

  if (strcmp (gps_device, "sjn") == 0) {
    strcpy (init_string, init_gps_sjn); }
  else if (strcmp (gps_device, "garmin_4800") == 0) {
    strcpy (init_string, init_gps_garmin_4800); }
  else if (strcmp (gps_device, "garmin_9600") == 0) {
    strcpy (init_string, init_gps_garmin_9600); }
  else if (strcmp (gps_device, "garmin_19200") == 0) {
    strcpy (init_string, init_gps_garmin_19200); }
  else if (strcmp (gps_device, "sirf_4800") == 0) {
    strcpy (init_string, init_gps_sirf_4800); }
  else if (strcmp (gps_device, "sirf_9600") == 0) {
    strcpy (init_string, init_gps_sirf_9600); }
  else if (strcmp (gps_device, "sirf_19200") == 0) {
    strcpy (init_string, init_gps_sirf_19200); }
  else if (strcmp (gps_device, "sirf_38400") == 0) {
    strcpy (init_string, init_gps_sirf_38400); }
  else if (gps_device[0] != 0) {
    printf ("Unknown gps ititialization string!\n");
    return; }
    
  fill_checksum (init_string);
  if (verbose_flag) {
    printf ("%s",init_string); }
  write (f, init_string, strlen(init_string));
}

/******************************************************************************/
/***** time functions *********************************************************/
/******************************************************************************/

// convert utc calender time to epoche nanoseconds
long long utc_ctime_to_ensec (int year, int mon, int day, int hour, int min, long double sec_nsec)
{
  int sec= (int)sec_nsec;
  long long nsec= (long long)(sec_nsec*1000000000ll)-sec*1000000000ll;
  struct tm t;
  t.tm_year= year-1900;
  t.tm_mon= mon-1;
  t.tm_mday= day;
  t.tm_hour= hour;
  t.tm_min= min;
  t.tm_sec= (int)sec;
  time_t esec= timegm(&t);
  return((long long)esec*1000000000ll+nsec);
}

// convert epoche nanoseconds to utc calender time
void ensec_to_utc_ctime (long long ensec, int *year, int *mon, int *day, int *hour, int *min, long double *sec_nsec)
{
  time_t esec= ensec/1000000000ll;
  struct tm *t = gmtime(&esec);
  *year= t->tm_year+1900;
  *mon= t->tm_mon+1;
  *day= t->tm_mday;
  *hour= t->tm_hour;
  *min= t->tm_min;
  *sec_nsec= t->tm_sec+(ensec%1000000000ll)/1000000000.0;
}

// return epoche nanoseconds
long long ensec_time ()
{
  struct timeval t;
  gettimeofday(&t,(struct timezone *)0);
  return((long long)(t.tv_sec)*1000000000ll+(long long)t.tv_usec*1000ll);
}

/******************************************************************************/
/***** open connection ********************************************************/
/******************************************************************************/

int open_connection ()
{
  struct hostent *hostinfo;
  int sockfd;

  if (verbose_flag) {
    printf ("Open connection\n"); }

  sockfd= socket (AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1) {
    exit (0); }

  sockaddr_in serv_addr;
  serv_addr.sin_family= AF_INET;
  serv_addr.sin_port= htons(STRIKE_SERVER_PORT);
  serv_addr.sin_addr.s_addr= inet_addr (STRIKES_SERVER_ADDR);
  if (serv_addr.sin_addr.s_addr == INADDR_NONE) {
    /* host not given by IP but by name */
    hostinfo= gethostbyname (STRIKES_SERVER_ADDR);
    if (hostinfo == NULL) {
      close (sockfd);
      exit (0); }
    memcpy((char*) &serv_addr.sin_addr.s_addr, hostinfo->h_addr, hostinfo->h_length); }

  if (connect(sockfd, (sockaddr *) &serv_addr, sizeof(sockaddr)) == -1) {
    exit (0); }

  return sockfd;
}

/******************************************************************************/
/***** send data **************************************************************/
/******************************************************************************/

void send_data ()
{
  int sockfd=  open_connection ();
  char buf [STRING_BUFFER_SIZE];

  if (verbose_flag) {
    printf ("Start sending\n"); }

  while (cnt3 != cnt2) {
    sprintf (buf, "%04d-%02d-%02d %02d:%02d:%02d.%09lld %.6Lf %.6Lf %.0Lf %s %s",
      strikes[cnt3].year, strikes[cnt3].mon, strikes[cnt3].day,
      strikes[cnt3].hour, strikes[cnt3].min, strikes[cnt3].sec, strikes[cnt3].nsec,
      strikes[cnt3].lat, strikes[cnt3].lon, strikes[cnt3].alt, username, password);
    sprintf (buf, "%s %.6Lf %.6Lf", buf, (long double)strikes[cnt3].A/(long double)AD_MAX_VALUE, (long double)strikes[cnt3].B/(long double)AD_MAX_VALUE);
    sprintf (buf, "%s %c %s\n", buf, strikes[cnt3].status, VERSION);
    if (send (sockfd, buf, strlen(buf), 0) == -1) {
      exit (0); }
    if (verbose_flag) {
      printf ("%s",buf); }
    cnt3++;
    cnt3%= STRIKE_BUFFER_SIZE; }

  send (sockfd, "\n", 2, 0);
  close (sockfd);

  exit (1);
}

/******************************************************************************/
/***** evaluate ***************************************************************/
/******************************************************************************/

void evaluate (char * buf, int f)
{
  int year, mon, day, hour, min, sec;
  int lat_deg, lon_deg, sat;
  long double lat_min, lon_min, alt;
  char ns, we, status;
  long long counter, difference;
  int A[64], B[64], best_A, best_B;

  if (sscanf (buf, "$BLSEC,%llx,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,M,%d",
      &counter, &status, &hour, &min, &sec, &day, &mon, &year, &lat_deg, &lat_min, &ns, &lon_deg, &lon_min, &we, &alt, &sat) == 16) {

    last_lat= lat_deg+lat_min/60.0;
    if (ns == 'S') {
      last_lat= -last_lat; }
    position_ring_buffer [ring_buffer_cnt].lat = last_lat;

    last_lon= lon_deg+lon_min/60.0;
    if (ns == 'W') {
      last_lon= -last_lon; }
    position_ring_buffer [ring_buffer_cnt].lon = last_lon;

    last_alt= alt;
    position_ring_buffer [ring_buffer_cnt].alt = last_alt;

    if (counter >  counter_last) {
      difference= counter-counter_last; }
    else {
      difference= (counter+0x1000000ll)-counter_last; }
    counter_last= counter;
    counter_ring_buffer [ring_buffer_cnt]= difference;

    ring_buffer_cnt= (ring_buffer_cnt+1)%RING_BUFFER_SIZE;

    counter_sum= 0;
    last_lat= 0.0;
    last_lon= 0.0;
    last_alt= 0.0;
    for (int i=0; i<RING_BUFFER_SIZE; i++) {
      counter_sum+= counter_ring_buffer [i];
      last_lat+= position_ring_buffer [i].lat;
      last_lon+= position_ring_buffer [i].lon;
      last_alt+= position_ring_buffer [i].alt; }
    counter_average= (long double)counter_sum/(long double)RING_BUFFER_SIZE;
    last_lat/= (long double)RING_BUFFER_SIZE;
    last_lon/= (long double)RING_BUFFER_SIZE;
    last_alt/= (long double)RING_BUFFER_SIZE;
    counter_precision= (counter_average-(long double)difference)/counter_average;

    if (verbose_flag) {
      printf ("counter: %7lld, accuracy of 1PPS signal %+12.9Lf sec\n", difference, counter_precision);
      printf ("averaged position: lat: %.6Lf, lon: %.6Lf, alt: %.1Lf, sat: %d\n", last_lat, last_lon, last_alt, sat); }

    if (fabsl(counter_precision) <= PRECISION) {
      cnt2= cnt1; }
    else {
      cnt1= cnt2; }

    last_year= year+2000;
    last_mon= mon;
    last_day= day;
    last_hour= hour;
    last_min= min;
    last_sec= sec;
    last_time= utc_ctime_to_ensec(last_year, last_mon, last_day, last_hour, last_min, (long double)last_sec);
    last_status= status;

    if ((last_time > last_time_send + sleep_time) && (cnt2 != cnt3)) {
      if (cld_pid > 0) {
        kill (cld_pid, SIGKILL); }
#ifdef KERNEL_2_4
      signal(SIGCHLD, SIG_IGN);  // Added to kill Children by ignoring SIGCHLD
#endif
      cld_pid= fork ();
      if (cld_pid == 0) {
        send_data (); }
      cnt3= cnt2;
      last_time_send= last_time; } }

  else if (sscanf (buf, "$BLSEQ,%6llx,", &counter) == 1) {

    int i= 0;
    buf+=14;
    while ((i<64) && (sscanf (buf, "%2x%2x", &A[i], &B[i]) == 2)) {
      i++;
      buf+=4; }

    if (counter >  counter_last) {
      difference= counter-counter_last; }
    else {
      difference= (counter+0x1000000ll)-counter_last; }

    long long last_nsec= (difference*RING_BUFFER_SIZE*1000000000ll)/counter_sum;

    int max_amplitude= 0;
    int max_index= 0;
    for (int i=0; i<64; i++) {
      A[i]-= AD_MAX_VALUE;
      B[i]-= AD_MAX_VALUE;
      int amplitude= A[i]*A[i]+B[i]*B[i];
      if (amplitude > max_amplitude) {
        max_amplitude= amplitude;
        best_A= A[i];
        best_B= B[i];
        max_index= i; } }
    if ((abs(A[max_index]) < AD_MAX_VALUE*AD_THRESHOLD_VOLTAGE/AD_MAX_VOLTAGE)&&
        (abs(B[max_index]) < AD_MAX_VALUE*AD_THRESHOLD_VOLTAGE/AD_MAX_VOLTAGE)) {
      best_A= AD_MAX_VALUE*AD_THRESHOLD_VOLTAGE/AD_MAX_VOLTAGE;
      best_B= 0;
      max_index= -1; }

    last_nsec+= (max_index-1)*3125ll; // time correction

    if (verbose_flag) {
      printf ("%7lld", difference);
      for (int i=0; i<64; i++) {
        printf (",%4d,%4d", A[i], B[i]); }
      printf (", max %d: %d %d\n", max_index, best_A, best_B); }

    if ((last_nsec >= 0) && (last_nsec < 1000000000ll)) {
      strikes [cnt1].year= last_year;
      strikes [cnt1].mon= last_mon;
      strikes [cnt1].day= last_day;
      strikes [cnt1].hour= last_hour;
      strikes [cnt1].min= last_min;
      strikes [cnt1].sec= last_sec;
      strikes [cnt1].nsec= last_nsec;
      strikes [cnt1].lat= last_lat;
      strikes [cnt1].lon= last_lon;
      strikes [cnt1].alt= last_alt;
      strikes [cnt1].status= last_status;
      strikes [cnt1].A= best_A;
      strikes [cnt1].B= best_B;
      cnt1++;
      cnt1%= STRIKE_BUFFER_SIZE; } }
}

/******************************************************************************/
/***** main *******************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
  char program_name [STRING_BUFFER_SIZE];
  program_name[0]= 0;
  if (argc >0) {
    strcpy(program_name,argv[0]);
    argc--;
    argv++; }

  sleep_time= SLEEP_TIME_SEC*1000000000ll;
  bool tty_device_flag= true;
  verbose_flag= false;
  char gps_device[STRING_BUFFER_SIZE];
  gps_device[0] = 0;
  int baud_rate= 0;
  int gps_init_day= 0;
  bool help_flag= false;


  bool flag_found;
  do {
    flag_found= false;
    if ((argc > 1) && (strcmp(argv[0],"-t") == 0)) {
      flag_found= true;
      sleep_time= atoll(argv[1])*1000000000ll;
      argc-= 2;
      argv+= 2; }
    if ((argc > 1) && (strcmp(argv[0],"-i") == 0)) {
      flag_found= true;
      strcpy (gps_device, argv[1]); 
      argc-= 2;
      argv+= 2; }
    if ((argc > 1) && (strcmp(argv[0],"-b") == 0)) {
      flag_found= true;
      baud_rate= atoi(argv[1]); 
      argc-= 2;
      argv+= 2; }
    if ((argc > 0) && (strcmp(argv[0],"-n") == 0)) {
      flag_found= true;
      tty_device_flag= false;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && (strcmp(argv[0],"-v") == 0)) {
      flag_found= true;
      verbose_flag= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && (strcmp(argv[0],"-h") == 0)) {
      flag_found= true;
      help_flag= true;
      argc--;
      argv++; } }
  while (flag_found);

  if ((help_flag)||(argc != 3)) {
    printf ("%s: [-t sleep_time] [-n] [-v] [-h] device username password\n",program_name);
    printf ("device     : serial device (example: /dev/ttyS0)\n");
    printf ("username   : username (example: PeterPim) \n");
    printf ("password   : password (example: xxxxxxxx)\n");
    printf ("-t sec     : sleep time (default = %d sec)\n",SLEEP_TIME_SEC);
    printf ("-i gps     : initialization with sjn,\n");
    printf ("             garmin_4800, garmin_9600, garmin_19200,\n");
    printf ("             sirf_4800, sirf_9600, sirf_19200, or sirf_38400\n");
    printf ("-b 0/1/2/3 : baud rate (0=4800, 1=9600, 2=19200, 3=38400)\n");
    printf ("-n         : device is not a tty\n");
    printf ("-v         : verbose mode\n");
    printf ("-h         : print this help text\n");
    exit (-1); }

  device= argv[0];
  username= argv[1];
  password= argv[2];

#ifndef KERNEL_2_4
  struct sigaction act;
  act.sa_handler = NULL;
  act.sa_flags = SA_NOCLDSTOP;
  sigaction(SIGCHLD,&act,NULL); 
#endif

  int f= open(device, O_RDWR | O_NOCTTY );
  if (f < 0) {
    printf("Can't open %s\n",device);
    exit(-1); }

  if (tty_device_flag) {
    struct termios tio;
    tio.c_iflag= IGNBRK | IGNPAR ;
    tio.c_oflag= OPOST | ONLCR ;
    if (baud_rate == 0) {
      tio.c_cflag= B4800 | CS8 | CLOCAL | CREAD ; }
    else if (baud_rate == 1) {
      tio.c_cflag= B9600 | CS8 | CLOCAL | CREAD ; }
    else if (baud_rate == 2) {
      tio.c_cflag= B19200 | CS8 | CLOCAL | CREAD ; }
    else if (baud_rate == 3) {
      tio.c_cflag= B38400 | CS8 | CLOCAL | CREAD ; }
    tio.c_lflag= 0;
    tio.c_cc[VTIME]= 0;
    tio.c_cc[VMIN]= 1;
    tcsetattr(f, TCSANOW, &tio); }

  char buf[STRING_BUFFER_SIZE], c;
  int i=0;

  int year, mon, day, hour, min;
  long double sec_nsec;

  while (true) {
    ensec_to_utc_ctime (ensec_time(), &year, &mon, &day, &hour, &min, &sec_nsec);
    if (day != gps_init_day) {
      init_gps (f, gps_device);
      gps_init_day= day; }

    if (read(f, &c, 1) == 1) {
      buf [i]= c;
      if (verbose_flag) {
        putchar (c); }
      if (i < STRING_BUFFER_SIZE-2) {
        i++; }
      if (c == '\n') {
        buf [i]= 0;
        evaluate (buf, f);
        i= 0; } } }
}

