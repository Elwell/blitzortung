/*
    Copyright (C) 2003-2012  Egon Wanke <blitzortung@gmx.org>

    This program sends the output of the evaluation board PCB 6 Version 8 or
    higher as udp packets to the servers of blitzortung.org. It supports only
    all firmware versions. If there are any problem with this tracker
    program, please contact me by email to blitzortung@gmx.org.
 
    Name this file "tracker_Linux.c" and compile it by
    > g++ -Wall -lm -o tracker_Linux tracker_Linux.c

    then try
    > ./tracker_Linux -h

    or
    > ./tracker_Linux --help



    Examples of the use for PCB 6 Version 8 with Firmware version 30a or higher:

    > ./tracker_Linux -vi -vl -vo -vs - /dev/ttyS0 CharlyAU yzyzyzyz 2
    This command writes sytstem information, log information, board output,
    and sent UDP packets to standard output,
    sets GPS speed to 4800 baud, does not initialize the GPS,
    sets tracker speed to 115200 baud, and uses serial device = /dev/ttyS0,
    username = CharlyAU, password = yzyzyzyz, and region = 2

    > ./tracker_Linux -bg 9600 SiRF /dev/ttyUSB0 PeterPim aaaaaaaa 1
    This command set GPS speed to 9600 baud, initializes the GPS with SiRF chipset,
    sets tracker speed to 115200 baud, and uses serial device = /dev/ttyUSB0,
    username = PeterPin, password = aaaaaaaa, and region = 1

    > ./tracker_Linux -vi -ll tracker.log -bt 500000 - /dev/ttyS0 CharlyAU yzyzyzyz 2
    This command outputs system information on standard output,
    writes log information to file tracker.log,
    sets GPS speed to 4800 baud, does not initialize the GPS,
    sets tracker speed to 500000 baud, and uses serial device = /dev/ttyS0,
    username = CharlyAU, password = yzyzyzyz, and region = 2

    If you want to controll the GPS output, then turn the yellow jumpers of the
    board by 90 degrees and type the following command:

    > ./tracker_Linux -vo -bg 4800 -bt 4800 SiRF /dev/ttyUSB0
    This command outputs board output on standard output,
    sets GPS speed to 4800 baud, intializes the GPS with SiRF chipset,
    sets tracker speed to GPS speed (= 4800 baud),
    and uses serial device = /dev/ttyUSB0



    Examples of the use for boards before PCB 6 Version 8 with Firmware version
    less than 30a. For these boards the tracker baudrate always has to be equal
    to the GPS baudrate:

    > ./tracker_Linux -vi -vl -vo -vs -bg 38400 -bt 38400 SiRF /dev/ttyS0 CharlyAU yzyzyzyz 1
    This command writes sytstem information, log information, board output,
    and sent UDP packets to standard output,
    sets GPS speed to 38400 baud, initializes the GPS,
    sets tracker speed to 38400 baud, and uses serial device = /dev/ttyS0,
    username = CharlyAU, password = yzyzyzyz, and region = 1

    To controll the GPS output, turn the yellow jumpers of the board by 90 degrees and
    type the following command which is the same for all boards:

    > ./tracker_Linux -vo -bg 4800 SiRF /dev/ttyUSB0
    This command outputs board output on standard output,
    sets GPS speed to 4800 baud, intializes the GPS with SiRF chipset,
    sets tracker speed to GPS speed (= 4800 baud),
    and uses serial device = /dev/ttyUSB0

*/

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
#include <syslog.h>
#include <math.h>

long double fabsl (long double x); // only necessary for OpenWrt,
                                   // does not disturb other systems

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#define VERSION                 "LT&nbsp;25"           // version string send to server
#define SERVER_ADDR_1           "rechenserver.de"      // server address region 1
#define SERVER_ADDR_2           "rechenserver.com"     // server address region 2
#define SERVER_ADDR_3           "rechenserver.org"     // server address region 3
#define SERVER_ADDR_4           "rechenserver.org"     // server address region 3
#define SERVER_PORT             8308                   // server port

const char *server_addr[]= {"0.0.0.0", SERVER_ADDR_1, SERVER_ADDR_2, SERVER_ADDR_3, SERVER_ADDR_4};

#define STRING_BUFFER_SIZE      2048                   // maximal buffer size for the strings we use to receive UDP packets
#define INFO_BUFFER_SIZE        128                    // maximal buffer size for the strings we use elsewhere
#define RING_BUFFER_SIZE        20                     // ring buffer size for averaged computation of counter difference
#define SMOOTH_FACTOR           3600                   // average of 3600 seconds
#define POS_PRECISION           0.001000l              // position precision in degree to reach before sending data
#define PPS_PRECISION           0.000001l              // pulse precision to reach before sending data

#define MAX_NONZERO_SEC         10                     // maximal number of consecutive seconds with strikes

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

//
struct flag_type {
  bool verbose_out;
  bool verbose_log;
  bool verbose_sent;
  bool verbose_info;
  bool SBAS;
  bool syslog;
  bool help; } flag;

//
struct precision_type {
  long double S_counter;
  long double lat;
  long double lon;
  long double alt;
  long double PPS; } precision;

//
struct ring_buffer_type {
  long long S_counter_difference;
  long double lat;
  long double lon;
  long double alt; } ring_buffer[RING_BUFFER_SIZE], average;

int ring_buffer_index= 0;

//
struct S_type {
  long long counter;
  char status;
  int year;
  int mon;
  int day;
  int hour;
  int min;
  int sec;
  long double lat;
  long double lon;
  long double alt;
  long double average_lat;
  long double average_lon;
  long double average_alt;
  int satellites;
  char firmware_version[INFO_BUFFER_SIZE];
  char tracker_version[INFO_BUFFER_SIZE]; } S, last_S;

//
struct L_type {
  long long counter;
  long long nsec;
  int channels;
  int values;
  int bits;
  bool found;
  char data[STRING_BUFFER_SIZE]; } L;

//
struct control_type {
  bool accuracy_ok;
  bool seconds_flow_ok;
  bool pos_ok;
  bool checksum_ok;
  bool faulty; } C, last_C;

int strikes_per_sec= 0;
int nonzero_sec= 0;
struct ring_buffer_type sum;
long long last_transmission_time= 0ll;

//
struct serial_type {
  char *device;
  char *echo_device;
  int tracker_baudrate;
  int gps_baudrate;
  char *gps_type;
  char tracker_baudrates_string[INFO_BUFFER_SIZE];
  char gps_baudrates_string[INFO_BUFFER_SIZE]; } serial;

#define DEFAULT_TRACKER_BAUDRATE        115200
#define DEFAULT_GPS_BAUDRATE            4800

//
struct logfile_type {
  char *name;
  FILE *fd; };

struct logfiles_type {
  struct logfile_type out;
  struct logfile_type log;
  struct logfile_type sent; } logfiles;

/******************************************************************************/
/***** initialization strings for gps chip sets *******************************/
/******************************************************************************/

//
// initialization for San Jose Navigation moduls, 4800 baud
//
char init_gps_SANAV[]="\
$PFEC,GPint,GGA01,GLL00,GSA00,GSV00,RMC01,DTM00,VTG00,ZDA00*00\n";

//
// initialization for Garmin moduls
//
char init_gps_Garmin[]= "\
$PGRMO,GPGGA,1*00\r\n\
$PGRMO,GPGSA,0*00\r\n\
$PGRMO,GPGSV,0*00\r\n\
$PGRMO,GPRMC,1*00\r\n\
$PGRMO,GPVTG,0*00\r\n\
$PGRMO,PGRMM,0*00\r\n\
$PGRMO,PGRMT,0*00\r\n\
$PGRMO,PGRME,0*00\r\n\
$PGRMO,PGRMB,0*00\r\n\
$PGRMCE*00\r\n";

char init_gps_Garmin_SBAS_on[]= "$PGRMC1,1,,,,,,,W,,,,,*00\r\n";
char init_gps_Garmin_SBAS_off[]= "$PGRMC1,1,,,,,,,N,,,,,*00\r\n";

char init_gps_Garmin_4800[]= "$PGRMC,,,,,,,,,,3,,2,4,*00\r\n";
char init_gps_Garmin_9600[]= "$PGRMC,,,,,,,,,,4,,2,4,*00\r\n";
char init_gps_Garmin_19200[]= "$PGRMC,,,,,,,,,,5,,2,4,*00\r\n";
char init_gps_Garmin_38400[]= "$PGRMC,,,,,,,,,,8,,2,4,*00\r\n";

//
// initialization for SiRF moduls
//
char init_gps_SiRF[]= "\
$PSRF103,00,00,01,01*00\r\n\
$PSRF103,01,00,00,01*00\r\n\
$PSRF103,02,00,00,01*00\r\n\
$PSRF103,03,00,00,01*00\r\n\
$PSRF103,04,00,01,01*00\r\n\
$PSRF103,05,00,00,01*00\r\n\
$PSRF103,06,00,00,01*00\r\n\
$PSRF103,08,00,00,01*00\r\n";

char init_gps_SiRF_SBAS_on[]= "$PSRF151,01*00\r\n";
char init_gps_SiRF_SBAS_off[]= "$PSRF151,00*00\r\n";

char init_gps_SiRF_4800[]= "$PSRF100,1,4800,8,1,0*00\r\n";
char init_gps_SiRF_9600[]= "$PSRF100,1,9600,8,1,0*00\r\n";
char init_gps_SiRF_19200[]= "$PSRF100,1,19200,8,1,0*00\r\n";
char init_gps_SiRF_38400[]= "$PSRF100,1,38400,8,1,0*00\r\n";

/******************************************************************************/
/***** time functions *********************************************************/
/******************************************************************************/

//
// convert utc calender time to epoche nanoseconds
//
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
  return ((long long)esec*1000000000ll+nsec);
}

//
// convert epoche nanoseconds to utc calender time
//
void ensec_to_utc_ctime (long long ensec, int *year, int *mon, int *day, int *hour, int *min, long double *sec_nsec)
{
  time_t esec= ensec/1000000000ll;
  struct tm *t = gmtime (&esec);
  *year= t->tm_year+1900;
  *mon= t->tm_mon+1;
  *day= t->tm_mday;
  *hour= t->tm_hour;
  *min= t->tm_min;
  *sec_nsec= t->tm_sec+(ensec%1000000000ll)/1000000000.0l;
}

//
// return epoche nanoseconds
//
long long ensec_time ()
{
  struct timeval t;
  gettimeofday (&t,(struct timezone *)0);
  return ((long long)(t.tv_sec)*1000000000ll+(long long)t.tv_usec*1000ll);
}

/******************************************************************************/
/***** write log message to log file ******************************************/
/******************************************************************************/

void write_log_message (const char *text)
{
  int year, mon, day, min, hour;
  long double sec_nsec;
  char buf [STRING_BUFFER_SIZE];

  ensec_to_utc_ctime (ensec_time(), &year, &mon, &day, &hour, &min, &sec_nsec);
  sprintf (buf, "%04d-%02d-%02d %02d:%02d:%02.0Lf, PID: %d,", year, mon, day, hour, min, sec_nsec, (int)getpid());

  if (logfiles.log.name != NULL) {
    fprintf (logfiles.log.fd, "%s %s", buf, text);
    fflush (logfiles.log.fd); }
  if (flag.verbose_log) {
    printf ("%s %s", buf, text);
    fflush (stdout); }
  if (flag.syslog) {
    openlog ("blitzortung", LOG_CONS|LOG_PID, LOG_USER);
    syslog (LOG_INFO, "%s", text);
    closelog (); }
}

/******************************************************************************/
/***** initialization *********************************************************/
/******************************************************************************/

//
// initialize serial struct
//
void init_struct_serial_type ()
{
  serial.tracker_baudrate= DEFAULT_TRACKER_BAUDRATE;
  serial.gps_baudrate= DEFAULT_GPS_BAUDRATE;
  serial.echo_device= NULL;
  serial.tracker_baudrates_string[0]= 0;
  strcat (serial.tracker_baudrates_string, "4800, 9600, 19200, 38400, 115200");
#ifdef B250000
  strcat (serial.tracker_baudrates_string, ", 250000");
#endif
#ifdef B500000
  strcat (serial.tracker_baudrates_string, ", 500000");
#endif
#ifdef B2500000
  strcat (serial.tracker_baudrates_string, ", 2500000");
#endif
  serial.gps_baudrates_string[0]= 0;
  strcat (serial.gps_baudrates_string, "4800, 9600, 19200, 38400");
}

//
// initialize flag struct
//
void init_struct_flag_type ()
{
  flag.verbose_out= false;
  flag.verbose_log= false;
  flag.verbose_sent= false;
  flag.verbose_info= false;
  flag.syslog= false;
  flag.SBAS= false;
  flag.help= false;
}

//
// initialize logfile struct
//
void init_struct_logfile_type ()
{
  logfiles.log.name= NULL;
  logfiles.log.fd= NULL;
  logfiles.out.name= NULL;
  logfiles.out.fd= NULL;
  logfiles.sent.name= NULL;
  logfiles.sent.fd= NULL;
}

//
// initialize S type
//
void init_struct_S_type ()
{
  S.counter= -1;
  S.status= '-';
  for (int i=0; i< INFO_BUFFER_SIZE; i++) {
    S.firmware_version[i]= 0;
    S.tracker_version[i]= 0; }
}

//
// initialize C type
//
void init_struct_C_type ()
{
  C.accuracy_ok= false;
  C.seconds_flow_ok= false;
  C.pos_ok= false;
  C.checksum_ok= false;
  C.faulty= false;
}

//
//
//
char int_to_hex (int b)
{
  b&= 0x0F;
  if (b > 9) {
    return (b+'A'-10); }
  else {
    return (b+'0'); }
}

//
//
//
char hex_to_int (char c)
{
  if ((c >= '0')&&(c <= '9')) {
    return (c-'0'); }
  else {
    return (c-'A'+10); }
}

//
// computes NMEA checksums between every $ and *
//
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

int compute_checksum (const char *buf)
{
  unsigned int i;
  int c= 0;

  for (i=0; i < strlen (buf); i++) {
    if (buf[i] == '$') {
      c= 0; }
    else if (buf[i] == '*') {
      return (c); }
    else {
      c^=buf[i]; } }
  return (c);
}

//
// set baudrate of open tty
//
void set_baudrate (int f, int baudrate)
{
  struct termios tio;
  tio.c_iflag= IGNBRK;
  tio.c_oflag= 0;
  tio.c_cflag= CS8 | CLOCAL | CREAD;
  if (baudrate == 4800) {
    tio.c_cflag|= B4800; }
  if (baudrate == 9600) {
    tio.c_cflag|= B9600; }
  if (baudrate == 19200) {
    tio.c_cflag|= B19200; }
  if (baudrate == 38400) {
    tio.c_cflag|= B38400; }
  if (baudrate == 115200) {
    tio.c_cflag|= B115200; }
#ifdef B250000
  if (baudrate == 250000) {
    tio.c_cflag|= B250000; }
#endif
#ifdef B500000
  if (baudrate == 500000) {
    tio.c_cflag|= B500000; }
#endif
#ifdef B2500000
  if (baudrate == 2500000) {
    tio.c_cflag|= B2500000; }
#endif
  tio.c_lflag= 0;
  tio.c_cc[VTIME]= 0;
  tio.c_cc[VMIN]= 1;
  tcsetattr (f, TCSANOW, &tio);
}

//
// initialize the GPS modul
//
void init_gps (struct serial_type serial)
{
  char init_string [STRING_BUFFER_SIZE];
  char buf [STRING_BUFFER_SIZE];

  if (strcmp (serial.gps_type, "SANAV") == 0) {
    strcpy (init_string, init_gps_SANAV);
    if (flag.SBAS) {
      fprintf (stderr, "Do not know how to initialize GPS chip set SANAV with SBAS support!\n");
      exit (EXIT_FAILURE); }
    if (serial.gps_baudrate != 4800) {
      fprintf (stderr, "Do not know how to initialize GPS chip set SANAV with %d Baud!\n", serial.gps_baudrate);
      exit (EXIT_FAILURE); } }

  else if (strcmp (serial.gps_type, "Garmin") == 0) {
    strcpy (init_string, init_gps_Garmin);
    if (serial.gps_baudrate == 4800) {
      strcat (init_string, init_gps_Garmin_4800); }
    else if (serial.gps_baudrate == 9600) {
      strcat (init_string, init_gps_Garmin_9600); }
    else if (serial.gps_baudrate == 19200) {
      strcat (init_string, init_gps_Garmin_19200); }
    else if (serial.gps_baudrate == 38400) {
      strcat (init_string, init_gps_Garmin_38400); }
    else {
      fprintf (stderr, "Do not know how to initialize GPS chip set Garmin with %d Baud!\n", serial.gps_baudrate);
      exit (EXIT_FAILURE); }

    if (flag.SBAS) {
      strcat (init_string, init_gps_Garmin_SBAS_on); }
    else {
      strcat (init_string, init_gps_Garmin_SBAS_off); } }

  else if (strcmp (serial.gps_type, "SiRF") == 0) {
    strcpy (init_string, init_gps_SiRF);
    if (serial.gps_baudrate == 4800) {
      strcat (init_string, init_gps_SiRF_4800); }
    else if (serial.gps_baudrate == 9600) {
      strcat (init_string, init_gps_SiRF_9600); }
    else if (serial.gps_baudrate == 19200) {
      strcat (init_string, init_gps_SiRF_19200); }
    else if (serial.gps_baudrate == 38400) {
      strcat (init_string, init_gps_SiRF_38400); }
    else {
      fprintf (stderr, "Do not know how to initialize GPS chip set SiRF with %d Baud!\n", serial.gps_baudrate);
      exit (EXIT_FAILURE); }
    if (flag.SBAS) {
      strcat (init_string, init_gps_SiRF_SBAS_on); }
    else {
      strcat (init_string, init_gps_SiRF_SBAS_off); } }

  else if (strcmp (serial.gps_type, "-") == 0) {
    return; }

  else {
    fprintf (stderr, "Do not know how to initialize GPS type %s!\n", serial.gps_type);
    fprintf (stderr, "Use SANAV, Garmin, SiRF, or - for no initialization.\n");
    exit (EXIT_FAILURE); }

  fill_checksum (init_string);
  write_log_message (init_string);

  int br= 38400;
  for (int b=0; b<4; b++) {
    int f= open (serial.device, O_RDWR | O_NOCTTY );
    if (f == -1) {
      sprintf (buf, "open (%s, O_RDWR | O_NOCTTY )", serial.device);
      perror (buf);
      exit (EXIT_FAILURE); }
    set_baudrate (f, br);
    sprintf (buf, "initialize GPS chip set %s, %d baud, using %d baud, please wait!\n", serial.gps_type, serial.gps_baudrate, br);
    write_log_message (buf);
    usleep (1000000);
    write (f, init_string, strlen(init_string));
    usleep (1000000);
    close (f);
    usleep (100000);
    br>>=1; }
  write_log_message ("Ready!\n");
}

/******************************************************************************/
/***** send_strike ************************************************************/
/******************************************************************************/

//
// send strike information
//
void send_strike (int sock_id, struct sockaddr *serv_addr, const char *username, const char *password)
{
  char buf [STRING_BUFFER_SIZE];
  if ((S.year < 2012)||(S.year > 2031)) {
    write_log_message ("error: year not between 2011 and 2031\n");
    return; }
  if ((S.mon < 1)||(S.mon > 12)) {
    write_log_message ("error: month not between 1 and 12\n");
    return; }
  if ((S.day < 1)||(S.day > 31)) {
    write_log_message ("error: day not between 1 and 31\n");
    return; }
  if ((S.hour < 0)||(S.hour > 23)) {
    write_log_message ("error: hour not between 0 and 23\n");
    return; }
  if ((S.min < 0)||(S.min > 59)) {
    write_log_message ("error: minutes not between 0 and 59\n");
    return; }
  if ((S.sec < 0)||(S.sec > 59)) {
    write_log_message ("error: seconds not between 0 and 59\n");
    return; }
  if ((L.nsec < 0)||(L.nsec > 999999999ll)) {
    write_log_message ("error: nanoseconds not between 0 and 999999999\n");
    return; }
  if ((S.average_lat < -90)||(S.average_lat > 90)) {
    write_log_message ("error: latitude not between -90 and 90\n");
    return; }
  if ((S.average_lon < -180)||(S.average_lon > 180)) {
    write_log_message ("error: latitude not between -180 and 180\n");
    return; }
  if ((S.average_alt < -1000)||(S.average_alt > 10000)) {
    write_log_message ("error: altitude not between -1000 and 10000\n");
    return; }
  if ((S.status != 'A')&&(S.status != 'V')) {
    write_log_message ("error: status not 'A' or 'V'\n");
    return; }

  sprintf (buf, "%04d-%02d-%02d %02d:%02d:%02d.%09lld %.6Lf %.6Lf %.0Lf %s %s %c %d %d %d %d %s %s %s %d/%d %s\n",
    S.year, S.mon, S.day, S.hour, S.min, S.sec, L.nsec,
    S.average_lat, S.average_lon, S.average_alt, username, password,
    S.status, S.satellites, L.channels, L.values, L.bits, L.data, VERSION, S.firmware_version,
    serial.gps_baudrate, serial.tracker_baudrate, serial.gps_type);

  if (sendto (sock_id, buf, strlen (buf), 0, serv_addr, sizeof (struct sockaddr)) == -1) {
    write_log_message ("error: sendto ()\n");
    return; }
  if (flag.verbose_sent) {
    printf ("%s", buf);
    fflush (stdout); }
  if (logfiles.sent.name != NULL) {
    fprintf (logfiles.sent.fd, "%s", buf);
    fflush (logfiles.sent.fd); }
}

/******************************************************************************/
/***** evaluate ***************************************************************/
/******************************************************************************/

//
// log status 
//
void log_status ()
{
  if ((last_C.faulty)&&(C.faulty))
    return;

  char buf [STRING_BUFFER_SIZE];

  if (!(C.checksum_ok)) {
    write_log_message ("Checksum error\n"); }

  if ((!(last_C.seconds_flow_ok))&&(C.seconds_flow_ok)) {
    sprintf (buf, "GPS seconds running, %d %d\n", last_S.sec, S.sec);
    write_log_message (buf); }
  if ((last_C.seconds_flow_ok)&&(!(C.seconds_flow_ok))) {
    sprintf (buf, "GPS seconds stopped, %d %d\n", last_S.sec, S.sec);
    write_log_message (buf); }

  if (last_S.status != S.status) {
    sprintf (buf, "GPS status changed from '%c' to '%c'\n", last_S.status, S.status);
    write_log_message (buf); }

  if ((!(last_C.pos_ok))&&(C.pos_ok)) {
    sprintf (buf, "Position fixed, lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf\n", S.average_lat, S.average_lon, S.average_alt);
    write_log_message (buf); }
  if ((last_C.pos_ok)&&(!(C.pos_ok))) {
    sprintf (buf, "Position lost, lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf\n", S.lat, S.lon, S.alt);
    write_log_message (buf); }

  if ((!(last_C.accuracy_ok))&&(C.accuracy_ok)) {
    sprintf (buf, "1PPS signal accuracy ok, counter: %06llX %06llX, %+.0Lf nsec\n", last_S.counter, S.counter, precision.PPS*1000000000.0l);
    write_log_message (buf); }
  if ((last_C.accuracy_ok)&&(!(C.accuracy_ok))) {
    sprintf (buf, "1PPS signal inaccurate, counter: %06llX %06llX, accuracy: %+.0LF nsec\n", last_S.counter, S.counter, precision.PPS*1000000000.0l);
    write_log_message (buf); }

  if ((!(last_C.faulty))&&(C.faulty)) {
    sprintf (buf, "interference mode started, %d strikes per second, %d seconds nonzero\n", strikes_per_sec, nonzero_sec);
    write_log_message (buf); }
  if ((last_C.faulty)&&(!(C.faulty))) {
    sprintf (buf, "interference mode stopped\n");
    write_log_message (buf); }
}

//
// evaluate received line
//
void evaluate (const char *line, int sock_id, struct sockaddr *serv_addr, const char *username, const char *password)
{
  int year, mon, day, min, hour, sec= 0, lat_deg, lon_deg, satellites= 0, no_param= 0;
  long double lat_min, lon_min, alt= 0.0l;
  char north_south, west_east, status;
  long long counter, counter_difference;
  char firmware_version [INFO_BUFFER_SIZE];
  firmware_version [0]= 0;
  char buf [STRING_BUFFER_SIZE];
  int checksum;
  int A, B;

  L.found= false;

  //
  // Second sentences
  //
  if ((((no_param= sscanf (line, "$S,%llX,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,%d,%3s*%x",
        &counter,
        &status,
        &hour, &min, &sec, &day, &mon, &year,
        &lat_deg, &lat_min, &north_south,
        &lon_deg, &lon_min, &west_east, &alt,
        &satellites,
        firmware_version,
        &checksum)) == 18) ||
       ((no_param= sscanf (line, "$BS,%llX,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,%d,%3s*%x",
        &counter,
        &status,
        &hour, &min, &sec, &day, &mon, &year,
        &lat_deg, &lat_min, &north_south,
        &lon_deg, &lon_min, &west_east, &alt,
        &satellites,
        firmware_version,
        &checksum)) == 18) ||
       ((no_param= sscanf (line, "$BLSEC,%llX,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,M,%d*%x",
        &counter,
        &status,
        &hour, &min, &sec, &day, &mon, &year,
        &lat_deg, &lat_min, &north_south,
        &lon_deg, &lon_min, &west_east, &alt,
        &satellites,
        &checksum)) == 17) ||
       ((no_param= sscanf (line, "$BLSEC,%2d%2d%2d,%2d%2d%2d,%c,%2d%Lf,%c,%3d%Lf,%c,%llX*%x",
        &hour, &min, &sec, &day, &mon, &year,
        &status,
        &lat_deg, &lat_min, &north_south,
        &lon_deg, &lon_min, &west_east,
        &counter,
        &checksum)) == 15)) &&
      (C.checksum_ok= (checksum == compute_checksum(line)))) {

    if (no_param == 15) {
      satellites= -1;
      alt= 0.0l;
      strcpy (firmware_version,"15a"); }
    if (no_param == 17) {
      strcpy (firmware_version,"19a"); }

    last_S= S;
    S.counter= counter;
    S.status= status;
    S.hour= hour;
    S.min= min;
    S.sec= sec;
    S.day= day;
    S.mon= mon;
    S.year= year+2000;
    S.lat= lat_deg+lat_min/60.0;
    if (north_south == 'S') {
      S.lat= -S.lat; }
    S.lon= lon_deg+lon_min/60.0;
    if (west_east == 'W') {
      S.lon= -S.lon; }
    S.alt= alt;
    S.satellites= satellites;
    strcpy (S.firmware_version,firmware_version);

    if (last_S.counter >= 0) {

      last_C= C;

      C.seconds_flow_ok= (S.sec == (last_S.sec+1)%60);

      counter_difference= S.counter-last_S.counter;
      if (counter_difference < 0) {
        counter_difference+= 0x1000000ll; }

      // fill the ring buffer
      ring_buffer [ring_buffer_index].S_counter_difference= counter_difference;
      ring_buffer [ring_buffer_index].lat= S.lat;
      ring_buffer [ring_buffer_index].lon= S.lon;
      ring_buffer [ring_buffer_index].alt= S.alt;

      ring_buffer_index= (ring_buffer_index+1)%RING_BUFFER_SIZE;

      // compute sums
      sum.S_counter_difference= 0;
      sum.lat= 0.0;
      sum.lon= 0.0;
      sum.alt= 0.0;
      for (int i=0; i<RING_BUFFER_SIZE; i++) {
        sum.S_counter_difference+= ring_buffer [i].S_counter_difference;
        sum.lat+= ring_buffer [i].lat;
        sum.lon+= ring_buffer [i].lon;
        sum.alt+= ring_buffer [i].alt; }

      average.S_counter_difference= sum.S_counter_difference/RING_BUFFER_SIZE;
      average.lat= sum.lat/RING_BUFFER_SIZE;
      average.lon= sum.lon/RING_BUFFER_SIZE;
      average.alt= sum.alt/RING_BUFFER_SIZE;

      precision.S_counter= average.S_counter_difference-counter_difference;
      precision.lat= fabsl(S.lat-average.lat);
      precision.lon= fabsl(S.lon-average.lon);
      precision.alt= S.alt-average.alt;
      precision.PPS= (long double)precision.S_counter/(long double)average.S_counter_difference;

      C.accuracy_ok= (fabsl (precision.PPS) < PPS_PRECISION);

      C.pos_ok= (precision.lat+precision.lon < POS_PRECISION);

      if (last_C.pos_ok) {
        S.average_lat= (S.average_lat*(SMOOTH_FACTOR-1)+average.lat)/SMOOTH_FACTOR;
        S.average_lon= (S.average_lon*(SMOOTH_FACTOR-1)+average.lon)/SMOOTH_FACTOR;
        S.average_alt= (S.average_alt*(SMOOTH_FACTOR-1)+average.alt)/SMOOTH_FACTOR; }
      else {
        S.average_lat= average.lat;
        S.average_lon= average.lon;
        S.average_alt= average.alt; }

      if (strikes_per_sec == 0) {
        nonzero_sec= 0;
        C.faulty= false; }
      else {
        nonzero_sec++;
        if (nonzero_sec >= MAX_NONZERO_SEC) {
          C.faulty= true; } }

      log_status();

      if (flag.verbose_info) {
        printf ("%Ld lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf, acc: %+.0Lf nsec, sat: %d\n", counter_difference, S.average_lat, S.average_lon, S.average_alt, precision.PPS*1000000000.0l, S.satellites);
        fflush (stdout); }

      long long time= ensec_time ();
      if ((C.accuracy_ok)&&(C.pos_ok)&&(C.seconds_flow_ok)&&
          (time > last_transmission_time + 600000000000ll)) {
        L.channels= 0;
        L.values= 0;
        L.bits= nonzero_sec;
        strcpy (L.data, "-");
        send_strike (sock_id, serv_addr, username, password);
        last_transmission_time= time; }

      strikes_per_sec= 0; } }

  //
  // Lightning sentence
  //
  // BLSIG sentence type 1
  else if ((sscanf (line, "$BLSIG,%6llx,%2x,%2x*%x", &L.counter, &A, &B, &checksum) == 4)&&(C.checksum_ok= (checksum == compute_checksum(line)))) {
    sprintf (L.data, "%02x%02x",A,B);
    L.channels= 2;
    L.values= 1;
    L.bits= 8;
    L.found= true; }

  // BLSIG sentence type 2
  else if ((sscanf (line, "$BLSIG,%6llx,%3x,%3x*%x", &L.counter, &A, &B, &checksum) == 4)&&(C.checksum_ok= (checksum == compute_checksum(line)))) {
    sprintf (L.data, "%03x%03x",A,B);
    L.channels= 2;
    L.values= 1;
    L.bits= 12;
    L.found= true; }

  // BLSIG sentence type 3
  else if ((sscanf (line, "$BLSEQ,%6llx,%256s*%x", &L.counter, L.data, &checksum) == 3)&&(C.checksum_ok= (checksum == compute_checksum(line)))) {
    L.channels= 2;
    L.values= 64;
    L.bits= 8;
    L.found= true; }

  // BD sentence type 1
  else if ((sscanf (line, "$BD,%6llx,%256s*%x", &L.counter, L.data, &checksum) == 3)&&(C.checksum_ok= (checksum == compute_checksum(line)))) {
    L.channels= 2;
    L.values= 64;
    L.bits= 8;
    L.found= true; }

  // BM sentence type 1
  else if ((sscanf (line, "$BM,%6llx,%256s*%x", &L.counter, L.data, &checksum) == 3)&&(C.checksum_ok= (checksum == compute_checksum(line)))) {
    L.channels= 1;
    L.values= 128;
    L.bits= 8;
    L.found= true; }

  // L sentence type 1
  else if ((sscanf (line, "$L,%6llx,%1024s*%x", &L.counter, L.data, &checksum) == 3)&&(C.checksum_ok= (checksum == compute_checksum(line)))) {
    L.channels= 2;
    L.values= 256;
    L.bits= 8;
    L.found= true; }

  // unknown sentence
  else {
    sprintf (buf, "unknown sentence: %s", line);
    write_log_message (buf); }

  //
  // send only if precision is ok
  //
  if (L.found) {
    strikes_per_sec++;
    if ((C.accuracy_ok)&&(C.pos_ok)&&(C.seconds_flow_ok)&&(!(C.faulty))) {
      counter_difference= L.counter-S.counter;
      if (counter_difference < 0) {
        counter_difference+= 0x1000000ll; }

      L.nsec= (long long)(counter_difference*RING_BUFFER_SIZE*1000000000ll)/sum.S_counter_difference;

      send_strike (sock_id, serv_addr, username, password);
      last_transmission_time= ensec_time (); } }
}

/******************************************************************************/
/***** main *******************************************************************/
/******************************************************************************/

int main (int argc, char **argv)
{
  char program_name [STRING_BUFFER_SIZE];
  program_name[0]= 0;
  if (argc >0) {
    strcpy (program_name,argv[0]);
    argc--;
    argv++; }

  char buf[STRING_BUFFER_SIZE];

  init_struct_serial_type ();
  init_struct_flag_type ();
  init_struct_logfile_type ();
  init_struct_S_type ();
  init_struct_C_type ();

  bool flag_found;
  do {
    flag_found= false;
    if ((argc > 0) && ((strcmp (argv[0],"-bt") == 0)||(strcmp (argv[0],"--tracker_baudrate") == 0))) {
      flag_found= true;
      serial.tracker_baudrate= atoi(argv[1]);
      argc-=2;
      argv+=2;
#ifndef B250000
      if (serial.tracker_baudrate == 250000) {
        fprintf (stderr, "Baudrate 250000 not possible with your hardware/driver, use %s!\n", serial.tracker_baudrates_string);
        exit (EXIT_FAILURE); }
#endif
#ifndef B500000
      if (serial.tracker_baudrate == 500000) {
        fprintf (stderr, "Baudrate 500000 not possible with your hardware/driver, use %s!\n", serial.tracker_baudrates_string);
        exit (EXIT_FAILURE); }
#endif
#ifndef B2500000
      if (serial.tracker_baudrate == 2500000) {
        fprintf (stderr, "Baudrate 2500000 not possible with your hardware/driver, use %s!\n", serial.tracker_baudrates_string);
        exit (EXIT_FAILURE); }
#endif
      if ((serial.tracker_baudrate != 4800)&&
          (serial.tracker_baudrate != 9600)&&
          (serial.tracker_baudrate != 19200)&&
          (serial.tracker_baudrate != 38400)&&
          (serial.tracker_baudrate != 115200)&&
          (serial.tracker_baudrate != 250000)&&
          (serial.tracker_baudrate != 500000)&&
          (serial.tracker_baudrate != 2500000)) {
        fprintf (stderr, "Baudrate %d not possible with your hardware/driver, user %s\n!", serial.tracker_baudrate, serial.tracker_baudrates_string);
        exit (EXIT_FAILURE); } }
    if ((argc > 0) && ((strcmp (argv[0],"-bg") == 0)||(strcmp (argv[0],"--gps_baudrate") == 0))) {
      flag_found= true;
      serial.gps_baudrate= atoi(argv[1]);
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0],"-L") == 0)||(strcmp (argv[0],"--syslog") == 0))) {
      flag_found= true;
      flag.syslog= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0], "-ll") == 0)|(strcmp (argv[0],"--log_log") == 0))) {
      flag_found= true;
      logfiles.log.name= argv[1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0], "-lo") == 0)|(strcmp (argv[0],"--log_output") == 0))) {
      flag_found= true;
      logfiles.out.name= argv[1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0], "-ls") == 0)|(strcmp (argv[0],"--log_sent") == 0))) {
      flag_found= true;
      logfiles.sent.name= argv[1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0],"-vl") == 0)||(strcmp (argv[0],"--verbose_log") == 0))) {
      flag_found= true;
      flag.verbose_log= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0],"-vi") == 0)||(strcmp (argv[0],"--verbose_info") == 0))) {
      flag_found= true;
      flag.verbose_info= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0],"-vo") == 0)||(strcmp (argv[0],"--verbose_output") == 0))) {
      flag_found= true;
      flag.verbose_out= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0],"-vs") == 0)||(strcmp (argv[0],"--verbose_sent") == 0))) {
      flag_found= true;
      flag.verbose_sent= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0], "-e") == 0)||(strcmp (argv[0],"--echo") == 0))) {
      flag_found= true;
      serial.echo_device= argv [1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0], "-s") == 0)||(strcmp (argv[0],"--SBAS") == 0))) {
      flag_found= true;
      flag.SBAS= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0],"-h") == 0)||(strcmp (argv[0], "--help") == 0))) {
      flag_found= true;
      flag.help= true;
      argc--;
      argv++; } }
  while (flag_found);

  if ((flag.help) || (!((argc == 2) || (argc == 5)))) {
    fprintf (stderr, "%s: [options] gps_type serial_device [username password region]\n", program_name);
    fprintf (stderr, "gps_type         : gps type (SANAV, Garmin, or SiRF ('-' for no initialization))\n");
    fprintf (stderr, "serial_device    : serial device (examples: /dev/ttyS0 or /dev/ttyUSB0)\n");
    fprintf (stderr, "username         : username (example: PeterPim) \n");
    fprintf (stderr, "password         : password (example: xxxxxxxx)\n");
    fprintf (stderr, "region           : region (1 = Europe, 2 = Oceanien, 3 = USA, 4 = Japan)\n");
    fprintf (stderr, "-bt baudrate     : tracker baudrate %s (default = %d)\n", serial.tracker_baudrates_string, DEFAULT_TRACKER_BAUDRATE);
    fprintf (stderr, "                   alternative: --baudrate\n");
    fprintf (stderr, "-bg baudrate     : gps baudrate %s (default = %d)\n", serial.gps_baudrates_string, DEFAULT_GPS_BAUDRATE);
    fprintf (stderr, "                   alternative: --baudrate_gpd\n");
    fprintf (stderr, "-L               : write log messages to the system message logger\n");
    fprintf (stderr, "                   altervative: --syslog\n");
    fprintf (stderr, "-ll file         : write log messages to file\n");
    fprintf (stderr, "                   alternative: --log_log file\n");
    fprintf (stderr, "-lo file         : write board output to file\n");
    fprintf (stderr, "                   alternative: --log_output file\n");
    fprintf (stderr, "-ls file         : write sent information to file\n");
    fprintf (stderr, "                   alternative: --log_sent file\n");
    fprintf (stderr, "-vl              : verbose mode, write log messages to stdout\n");
    fprintf (stderr, "                   alternative: --verbose_log\n");
    fprintf (stderr, "-vi              : verbose mode, write system information to stdout\n");
    fprintf (stderr, "                   alternative: --verbose_info\n");
    fprintf (stderr, "-vo              : verbose mode, write board output to stdout\n");
    fprintf (stderr, "                   alternative: --verbose_output\n");
    fprintf (stderr, "-vs              : verbose mode, write sent information to stdout\n");
    fprintf (stderr, "                   alternative: --verbose_sent\n");
    fprintf (stderr, "-e serial_device : serial device for input echo\n");
    fprintf (stderr, "                   alternative: --echo serial_device\n");
    fprintf (stderr, "-s               : activate SBAS (WAAS/EGNOS/MSAS) support\n");
    fprintf (stderr, "                   alternative: --SBAS\n");
    fprintf (stderr, "-h               : print this help text\n");
    fprintf (stderr, "                   alternative: --help\n");
    exit (EXIT_FAILURE); }

//
  char *username;
  char *password;
  int region;
  if (argc == 5) {
    username= argv[2];
    password= argv[3];
    region= atoi(argv[4]);
    if (strcmp (password, "xxxxxxxx") == 0) {
      fprintf (stderr, "The default password \"xxxxxxxx\" can not be used, please change your password!\n");
      exit (EXIT_FAILURE); }
    if ((region < 1) || (region > 4)) {
      fprintf (stderr, "Illegal region (1 = Europe, 2 = Oceanien, 3 = USA, 4 = Japan)!\n");
      exit (EXIT_FAILURE); } }

//
  if (logfiles.log.name != NULL) {
    logfiles.log.fd= fopen(logfiles.log.name, "w");
    if (logfiles.log.fd < 0) {
      sprintf (buf, "fopen (%s, \"w\")", logfiles.out.name);
      perror (buf);
      exit (EXIT_FAILURE); } }
  if (logfiles.out.name != NULL) {
    logfiles.out.fd= fopen(logfiles.out.name, "w");
    if (logfiles.out.fd < 0) {
      sprintf (buf, "fopen (%s, \"w\")", logfiles.out.name);
      perror (buf);
      exit (EXIT_FAILURE); } }
  if (logfiles.sent.name != NULL) {
    logfiles.sent.fd= fopen(logfiles.sent.name, "w");
    if (logfiles.sent.fd < 0) {
      sprintf (buf, "fopen (%s, \"w\")", logfiles.sent.name);
      perror (buf);
      exit (EXIT_FAILURE); } }
  write_log_message ("tracker started\n");

//
  serial.gps_type= argv[0];
  serial.device= argv[1];
  init_gps (serial);

//
  int e= 0;
  if (serial.echo_device != NULL) {
    e= open (serial.echo_device, O_RDWR | O_NOCTTY );
    if (e < 0) {
      sprintf (buf, "open (%s, O_RDWR | O_NOCTTY )", serial.echo_device);
      perror (buf);
      exit (EXIT_FAILURE); }
    set_baudrate (e, serial.tracker_baudrate); }

//
  int f= open (serial.device, O_RDWR | O_NOCTTY );
  if (f < 0) {
    sprintf (buf, "open (%s, O_RDWR | O_NOCTTY )", serial.device);
    perror (buf);
    exit (EXIT_FAILURE); }
  set_baudrate (f, serial.tracker_baudrate);

  unsigned char c;

  if (argc == 2) {

    while (true) {
      if (read (f, &c, 1) == 1) {
        if (serial.echo_device != NULL) {
          write (e, &c, 1); }
        if (flag.verbose_out) {
          putchar (c); }
        if (logfiles.out.name != NULL) {
          fwrite (&c, 1, 1, logfiles.out.fd);
          fflush (logfiles.out.fd); } } } }

  else {

    int sock_id= socket (AF_INET, SOCK_DGRAM, 0);
    if (sock_id == -1) {
      perror ("socket (AF_INET, SOCK_DGRAM, 0)");
      exit (EXIT_FAILURE); }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family= AF_INET;
    serv_addr.sin_port= htons (SERVER_PORT);
    serv_addr.sin_addr.s_addr= inet_addr (server_addr[region]);

    if (serv_addr.sin_addr.s_addr == INADDR_NONE) {
      /* host not given by IP but by name */
      struct hostent *host_info= gethostbyname (server_addr[region]);
      if (host_info == NULL) {
        sprintf (buf, "gethostbyname (%s) failed, try again in 10 seconds!\n", server_addr[region]);
        write_log_message (buf);
        sleep (10);
        struct hostent *host_info= gethostbyname (server_addr[region]);
        if (host_info == NULL) {
          sprintf (buf, "gethostbyname (%s) failed, try again in 60 seconds!\n", server_addr[region]);
          write_log_message (buf);
          sleep (10);
          struct hostent *host_info= gethostbyname (server_addr[region]);
          if (host_info == NULL) {
            sprintf (buf, "gethostbyname (%s) failed, give up, please check your internet connection!\n", server_addr[region]);
            write_log_message (buf);
            sprintf (buf, "gethostbyname (%s)",server_addr[region]);
            perror (buf);
            close (sock_id);
            exit (EXIT_FAILURE); } } }
       memcpy((char*) &serv_addr.sin_addr.s_addr, host_info->h_addr, host_info->h_length); }

    int i=0;

    do {
      read (f, &c, 1); }
    while (c != '\n');

    while (true) {
      if (read (f, &c, 1) == 1) {
        if (((c < ' ')||(c > '~'))&&(c != '\n')) {
          char hex[INFO_BUFFER_SIZE];
          sprintf (hex, "(0x%02X)", c);
          if (serial.echo_device != NULL) {
            write (e, hex, 6); }
          if (flag.verbose_out) {
            fwrite (hex, 6, 1, stdout);
            fflush (stdout); }
          if (logfiles.out.name != NULL) {
            fwrite (hex, 6, 1, logfiles.out.fd);
            fflush (logfiles.out.fd); } }
        else {
          if (serial.echo_device != NULL) {
            write (e, &c, 1); }
          if (flag.verbose_out) {
            fwrite (&c, 1, 1, stdout);
            fflush (stdout); }
          if (logfiles.out.name != NULL) {
            fwrite (&c, 1, 1, logfiles.out.fd);
            fflush (logfiles.out.fd); }
          buf [i]= c;
          if (i < STRING_BUFFER_SIZE-2) {
            i++; }
          if (c == '\n') {
            buf [i]= 0;
            evaluate (buf, sock_id, (struct sockaddr *)&serv_addr, username, password);
            i= 0; } } } } }

  close (e);
  close (f);
  exit(EXIT_SUCCESS);
}
