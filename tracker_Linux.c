/*
    Copyright (C) 2003-2011  Egon Wanke <blitzortung@gmx.org> 1

    This program sends the output of the evaluation boards as udp packets
    to the server of blitzortung.org. It supports all firmware version.
    If there are any problem with this tracker program, contact me by
    email to a blitzortung@gmx.org.
 
    Name this file "tracker_Linux.c" and compile it by
    > g++ -Wall -lm -o tracker_Linux tracker_Linux.c

    and try
    > ./tracker_Linux -h

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

long double fabsl (long double x); // this is only necessary for the OpenWrt
                                   // but do not disturb other systems

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#define VERSION                 "LT&nbsp;23"           // version string send to server
#define SERVER_ADDR             "rechenserver.de"      // server address
#define SERVER_PORT             8308                   // server port
#define STRING_BUFFER_SIZE      2048                   // maximal buffer size for the strings we use
#define RING_BUFFER_SIZE        20                     // ring buffer size for averaged computation of counter difference
#define SMOOTH_FACTOR           3600                   //
#define POS_PRECISION           0.001000l              // position precision in degree to reach before sending data
#define TIME_PRECISION          0.000001l              // time precision to reach before sending data

#define MAX_STR_SEC             12                     // maximal number of strikes per second
#define MAX_NONZERO_SEC         6                      // maximal number of seconds with strikes

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

// if verbose flags are true, the program outputs information on standard output
bool verbose_log_flag;
bool verbose_info_flag;
bool verbose_output_flag;
bool verbose_sent_flag;

// if logfile flags are true, the program outputs information in a file
bool logfile_log_flag= false;
bool logfile_output_flag= false;
bool logfile_sent_flag= false;

// if syslog flag is true, the program uses the system logger for system information
bool syslog_flag= false;

FILE *logfile_log_fd;
FILE *logfile_output_fd;
FILE *logfile_sent_fd;

// all these global last and last_last variables store the information of the
// last BLSEC sentence
long long last_counter= -1, last_last_counter= -1;
int last_year, last_mon, last_day, last_hour, last_min, last_sec, last_last_sec;
int last_sat, last_last_sat;
int last_channels;
int last_values;
int last_bits;
long double last_lat, last_last_lat;
long double last_lon, last_last_lon;
long double last_alt, last_last_alt;
long long last_nsec;
char last_status= '-', last_last_status;
bool last_pos_ok= false, last_last_pos_ok= false;
bool last_accuracy_ok= false, last_last_accuracy_ok= false;
bool last_sec_ok= false, last_last_sec_ok= false;
bool last_imode= false, last_last_imode= false;
char last_data[STRING_BUFFER_SIZE];
char last_firmware[STRING_BUFFER_SIZE];
long long last_transmition_time= 0ll;
bool last_checksum_error= false;

long long now_time= 0ll;
long long dif_sum= 1;
long double lat_sum;
long double lon_sum;
long double alt_sum;
long double dif_average;
long double lat_average;
long double lon_average;
long double alt_average;
long double lat_average_x;
long double lon_average_x;
long double alt_average_x;
long double dif_precision;
long double lat_precision;
long double lon_precision;
long double alt_precision;
long double time_precision;
int ring_buffer_index= 0;

// the ring buffer is used to compute the averred position and time difference
// between two BLSEC sentences
struct ring_buffer_type {
  long long dif;
  long double lat;
  long double lon;
  long double alt; } ring_buffer[RING_BUFFER_SIZE];

int str_sec= 0;
int last_str_sec= 0;
int nonzero_sec= 0;
int baudrate;
char *gps_type;

/******************************************************************************/
/***** initialization string for the gps devices ******************************/
/******************************************************************************/

// initialization for San Jose Navigation moduls, 4800 baud
char init_gps_SANAV[]="\
$PFEC,GPint,GGA01,GLL00,GSA00,GSV00,RMC01,DTM00,VTG00,ZDA00*00\n";

// initialization for Garmin moduls
char init_gps_Garmin[]= "\
$PGRMO,GPGGA,1*00\n\
$PGRMO,GPGSA,0*00\n\
$PGRMO,GPGSV,0*00\n\
$PGRMO,GPRMC,1*00\n\
$PGRMO,GPVTG,0*00\n\
$PGRMO,PGRMM,0*00\n\
$PGRMO,PGRMT,0*00\n\
$PGRMO,PGRME,0*00\n\
$PGRMO,PGRMB,0*00\n\
$PGRMCE*00\n";

char init_gps_Garmin_SBAS_on[]= "$PGRMC1,1,,,,,,,W,,,,,*00\n";
char init_gps_Garmin_SBAS_off[]= "$PGRMC1,1,,,,,,,N,,,,,*00\n";

char init_gps_Garmin_4800[]= "$PGRMC,,,,,,,,,,3,,2,4,*00\n";
char init_gps_Garmin_9600[]= "$PGRMC,,,,,,,,,,4,,2,4,*00\n";
char init_gps_Garmin_19200[]= "$PGRMC,,,,,,,,,,5,,2,4,*00\n";
char init_gps_Garmin_38400[]= "$PGRMC,,,,,,,,,,8,,2,4,*00\n";

// initialization for SiRF moduls
char init_gps_SiRF[]= "\
$PSRF103,00,00,01,01*00\n\
$PSRF103,01,00,00,01*00\n\
$PSRF103,02,00,00,01*00\n\
$PSRF103,03,00,00,01*00\n\
$PSRF103,04,00,01,01*00\n\
$PSRF103,05,00,00,01*00\n\
$PSRF103,06,00,00,01*00\n\
$PSRF103,08,00,00,01*00\n";

char init_gps_SiRF_SBAS_on[]= "$PSRF151,01*00\n";
char init_gps_SiRF_SBAS_off[]= "$PSRF151,00*00\n";

char init_gps_SiRF_4800[]= "$PSRF100,1,4800,8,1,0*00\n";
char init_gps_SiRF_9600[]= "$PSRF100,1,9600,8,1,0*00\n";
char init_gps_SiRF_19200[]= "$PSRF100,1,19200,8,1,0*00\n";
char init_gps_SiRF_38400[]= "$PSRF100,1,38400,8,1,0*00\n";

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
  return ((long long)esec*1000000000ll+nsec);
}

// convert epoche nanoseconds to utc calender time
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

// return epoche nanoseconds
long long ensec_time ()
{
  struct timeval t;
  gettimeofday (&t,(struct timezone *)0);
  return ((long long)(t.tv_sec)*1000000000ll+(long long)t.tv_usec*1000ll);
}

/******************************************************************************/
/***** write log messages to log file ***************************************/
/******************************************************************************/

void write_log_messages (const char *text)
{
  int year, mon, day, min, hour;
  long double sec_nsec;
  char buf [STRING_BUFFER_SIZE];

  ensec_to_utc_ctime (ensec_time(), &year, &mon, &day, &hour, &min, &sec_nsec);
  sprintf (buf, "%04d-%02d-%02d %02d:%02d:%02.0Lf, PID: %d,", year, mon, day, hour, min, sec_nsec, (int)getpid());

  if (logfile_log_flag) {
    fprintf (logfile_log_fd, "%s %s", buf, text);
    fflush (logfile_log_fd); }
  if (verbose_log_flag) {
    printf ("%s %s", buf, text);
    fflush (stdout); }
  if (syslog_flag) {
    openlog ("blitzortung", LOG_CONS|LOG_PID, LOG_USER);
    syslog (LOG_INFO, "%s", text);
    closelog (); }
}

/******************************************************************************/
/***** initialization *********************************************************/
/******************************************************************************/

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
char hex_to_int (char c)
{
  if ((c >= '0')&&(c <= '9')) {
    return (c-'0'); }
  else {
    return (c-'A'+10); }
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

// set baudrate of open tty
void set_baudrate (int f, int baudrate)
{
  struct termios tio;
  tio.c_iflag= IGNBRK | IGNPAR ;
  tio.c_oflag= OPOST | ONLCR ;
  if (baudrate == 300) {
    tio.c_cflag= B300 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 600) {
    tio.c_cflag= B600 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 1200) {
    tio.c_cflag= B1200 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 2400) {
    tio.c_cflag= B2400 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 4800) {
    tio.c_cflag= B4800 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 9600) {
    tio.c_cflag= B9600 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 19200) {
    tio.c_cflag= B19200 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 38400) {
    tio.c_cflag= B38400 | CS8 | CLOCAL | CREAD ; }
  else {
    printf ("Do not know how to initialize the tty with %d baud!\n", baudrate);
    exit (-1); }
  tio.c_lflag= 0;
  tio.c_cc[VTIME]= 0;
  tio.c_cc[VMIN]= 1;
  tcsetattr (f, TCSANOW, &tio);
}

// initialize the GPS modul
void init_gps (const char *serial_device, char *gps_type, int baudrate, bool SBAS_flag)
{
  char init_string [STRING_BUFFER_SIZE];
  char buf [STRING_BUFFER_SIZE];

  if (strcmp (gps_type, "SANAV") == 0) {
    strcpy (init_string, init_gps_SANAV);
    if (baudrate != 4800) {
      printf ("Do not know how to initialize GPS device SANAV with %d baud!\n", baudrate);
      exit (-1); }
    if (SBAS_flag == 1) {
      printf ("Do not know how to initialize GPS device SANAV with SBAS support!\n");
      exit (-1); } }

  else if (strcmp (gps_type, "Garmin") == 0) {
    strcpy (init_string, init_gps_Garmin);
    if (baudrate == 4800) {
      strcat (init_string, init_gps_Garmin_4800); }
    else if (baudrate == 9600) {
      strcat (init_string, init_gps_Garmin_9600); }
    else if (baudrate == 19200) {
      strcat (init_string, init_gps_Garmin_19200); }
    else if (baudrate == 38400) {
      strcat (init_string, init_gps_Garmin_38400); }
    else {
      printf ("Do not know how to initialize GPS device Garmin with %d baud!\n", baudrate);
      exit (-1); }
    if (SBAS_flag) {
      strcat (init_string, init_gps_Garmin_SBAS_on); }
    else {
      strcat (init_string, init_gps_Garmin_SBAS_off); } }

  else if (strcmp (gps_type, "SiRF") == 0) {
    strcpy (init_string, init_gps_SiRF);
    if (baudrate == 4800) {
      strcat (init_string, init_gps_SiRF_4800); }
    else if (baudrate == 9600) {
      strcat (init_string, init_gps_SiRF_9600); }
    else if (baudrate == 19200) {
      strcat (init_string, init_gps_SiRF_19200); }
    else if (baudrate == 38400) {
      strcat (init_string, init_gps_SiRF_38400); }
    else {
      printf ("Do not know how to initialize GPS device SiRF with %d baud!\n", baudrate);
      exit (-1); }
    if (SBAS_flag) {
      strcat (init_string, init_gps_SiRF_SBAS_on); }
    else {
      strcat (init_string, init_gps_SiRF_SBAS_off); } }

  else if (strcmp (gps_type, "-") == 0) {
    return; }

  fill_checksum (init_string);
  write_log_messages (init_string);

  int br= 4800;
  for (int b=0; b<4; b++) {
    int f= open (serial_device, O_RDWR | O_NOCTTY );
    if (f == -1) {
      perror ("open()");
      exit (-1); }
    set_baudrate (f, br);
    sprintf (buf, "initialize GPS with %s, %d baud, using %d baud\n", gps_type, baudrate, br);
    write_log_messages (buf);
    br+= br;
    write (f, "55555\n55555\n55555\n", 18); // only for synchronization
    write (f, init_string, strlen(init_string));
    close (f); }

}

/******************************************************************************/
/***** send_strike ************************************************************/
/******************************************************************************/

void send_strike (int sock_id, struct sockaddr *serv_addr, const char *username, const char *password)
{
  char buf [STRING_BUFFER_SIZE];
  sprintf (buf, "%04d-%02d-%02d %02d:%02d:%02d.%09lld %.6Lf %.6Lf %.0Lf %s %s %c %d %d %d %d %s %s %s %d %s\n",
    last_year, last_mon, last_day, last_hour, last_min, last_sec, last_nsec, lat_average_x, lon_average_x, alt_average_x, username, password, last_status, last_sat, last_channels, last_values, last_bits, last_data, VERSION, last_firmware, baudrate, gps_type);

  if (sendto (sock_id, buf, strlen (buf), 0, serv_addr, sizeof (sockaddr)) == -1) {
    write_log_messages ("error: sendto ()\n");
    return; }
  if (verbose_sent_flag) { 
    printf ("%s", buf);
    fflush (stdout); }
  if (logfile_sent_flag) { 
    fprintf (logfile_sent_fd, "%s", buf);
    fflush (logfile_sent_fd); }
}

/******************************************************************************/
/***** evaluate ***************************************************************/
/******************************************************************************/

// write status to log file
void log_status ()
{
  if ((last_last_imode)&&(last_imode))
    return;

  char buf [STRING_BUFFER_SIZE];

  if (last_checksum_error) {
    write_log_messages ("Checksum error\n"); }

  if ((!(last_last_sec_ok))&&(last_sec_ok)) {
    sprintf (buf, "GPS seconds running, %d %d\n", last_last_sec, last_sec);
    write_log_messages (buf); }
  if ((last_last_sec_ok)&&(!(last_sec_ok))) {
    sprintf (buf, "GPS seconds stopped, %d %d\n", last_last_sec, last_sec);
    write_log_messages (buf); }

  if (last_status != last_last_status) {
    sprintf (buf, "GPS status changed from '%c' to '%c'\n", last_last_status, last_status);
    write_log_messages (buf); }

  if ((!(last_last_pos_ok))&&(last_pos_ok)) {
    sprintf (buf, "Position fixed, lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf\n", lat_average, lon_average, alt_average);
    write_log_messages (buf); }
  if ((last_last_pos_ok)&&(!(last_pos_ok))) {
    sprintf (buf, "Position lost, lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf\n", last_lat, last_lon, last_alt);
    write_log_messages (buf); }

//  if (last_last_sat != last_sat) {
//    sprintf (buf, "Number of satellites changed from %d to %d\n", last_last_sat, last_sat);
//    write_log_messages (buf); }

  if ((!(last_last_accuracy_ok))&&(last_accuracy_ok)) {
    sprintf (buf, "1PPS signal accuracy ok, counter: %06llX %06llX, %+.0Lf nsec\n", last_last_counter, last_counter, time_precision*1000000000.0l);
    write_log_messages (buf); }
  if ((last_last_accuracy_ok)&&(!(last_accuracy_ok))) {
    sprintf (buf, "1PPS signal inaccurate, counter: %06llX %06llX, accuracy: %+.0LF nsec\n", last_last_counter, last_counter, time_precision*1000000000.0l);
    write_log_messages (buf); }

  if ((!(last_last_imode))&&(last_imode)) {
    sprintf (buf, "interference mode started, %d strikes per seconds, %d seconds nonzero\n", last_str_sec, nonzero_sec);
    write_log_messages (buf); }
  if ((last_last_imode)&&(!(last_imode))) {
    sprintf (buf, "interference mode stopped\n");
    write_log_messages (buf); }
}

void evaluate (const char *line, int sock_id, struct sockaddr *serv_addr, const char *username, const char *password)
{
  int year, mon, day, min, hour, sec= 0, lat_deg, lon_deg, sat= 0, A, B, n;
  long double lat, lat_min, lon, lon_min, alt= 0.0l;
  char ns, we, status;
  long long counter, dif;
  bool BLSIG_found= false;
  char buf[STRING_BUFFER_SIZE];
  char firmware[3];
  firmware [0]= '-';
  firmware [1]= '-';
  firmware [2]= '-';
  int checksum;

  // BLSEC sentences
  if ((((n= sscanf (line, "$BS,%llX,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,%d,%c%c%c*%X",
      &counter, &status, &hour, &min, &sec, &day, &mon, &year, &lat_deg, &lat_min, &ns, &lon_deg, &lon_min, &we, &alt,  &sat, &firmware[0], &firmware[1], &firmware[2], &checksum)) == 20) ||
       ((n= sscanf (line, "$BLSEC,%llX,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,M,%d*%X",
      &counter, &status, &hour, &min, &sec, &day, &mon, &year, &lat_deg, &lat_min, &ns, &lon_deg, &lon_min, &we, &alt, &sat, &checksum)) == 17) ||
       ((n= sscanf (line, "$BLSEC,%2d%2d%2d,%2d%2d%2d,%c,%2d%Lf,%c,%3d%Lf,%c,%llX*%X",
      &hour, &min, &sec, &day, &mon, &year, &status, &lat_deg, &lat_min, &ns, &lon_deg, &lon_min, &we, &counter, &checksum)) == 15))&&
      (!(last_checksum_error= (checksum != compute_checksum(line))))) {

    lat= lat_deg+lat_min/60.0;
    if (ns == 'S') {
      lat= -lat; }

    lon= lon_deg+lon_min/60.0;
    if (ns == 'W') {
      lon= -lon; }

    if (last_counter == -1) {
      last_lat= lat;
      last_lon= lon;
      last_alt= alt;
      last_counter= counter; }

    else {
      last_year= year+2000;
      last_mon= mon;
      last_day= day;
      last_min= min;
      last_hour= hour;
      last_last_sec_ok= last_sec_ok;
      last_sec_ok= (sec == (last_sec+1)%60);
      last_last_sec= last_sec;
      last_sec= sec;
      last_last_status= last_status;
      last_status= status;
      last_last_sat= last_sat;
      last_sat= sat;
      last_last_lat= last_lat;
      last_lat= lat;
      last_last_lon= last_lon;
      last_lon= lon;
      last_last_alt= last_alt;
      last_alt= alt;
      if (counter >  last_counter) {
        dif= counter-last_counter; }
      else {
        dif= (counter+0x1000000ll)-last_counter; }
      last_last_counter= last_counter;
      last_counter= counter;

      // firmware
      if (n == 20) {
        last_firmware[0]= firmware[0];
        last_firmware[1]= firmware[1];
        last_firmware[2]= firmware[2];
        last_firmware[3]= 0; }

      // fill the ring buffer
      ring_buffer [ring_buffer_index].lat= lat;
      ring_buffer [ring_buffer_index].lon= lon;
      ring_buffer [ring_buffer_index].alt= alt;
      ring_buffer [ring_buffer_index].dif= dif;

      ring_buffer_index= (ring_buffer_index+1)%RING_BUFFER_SIZE;

      // compute average dif, lat lon, alt for buffer 1
      dif_sum= 0;
      lat_sum= 0.0;
      lon_sum= 0.0;
      alt_sum= 0.0;
      for (int i=0; i<RING_BUFFER_SIZE; i++) {
        dif_sum+= ring_buffer [i].dif;
        lat_sum+= ring_buffer [i].lat;
        lon_sum+= ring_buffer [i].lon;
        alt_sum+= ring_buffer [i].alt; }
      dif_average= (long double)dif_sum/RING_BUFFER_SIZE;
      lat_average= lat_sum/RING_BUFFER_SIZE;
      lon_average= lon_sum/RING_BUFFER_SIZE;
      alt_average= alt_sum/RING_BUFFER_SIZE;

      dif_precision= (dif_average-dif);
      lat_precision= (lat_average-lat);
      lon_precision= (lon_average-lon);
      alt_precision= (alt_average-alt);
      time_precision= (dif_average-dif)/dif_average;

      last_last_accuracy_ok= last_accuracy_ok;
      last_accuracy_ok= (fabsl (time_precision) < TIME_PRECISION);

      last_last_pos_ok= last_pos_ok;
      last_pos_ok= (fabsl (lat_precision) + fabsl (lon_precision) < POS_PRECISION);
      if ((!(last_last_pos_ok))&&(last_pos_ok)) {
        lat_average_x= lat_average;
        lon_average_x= lon_average;
        alt_average_x= alt_average; }
      else {
        lat_average_x= (lat_average_x*(SMOOTH_FACTOR-1)+lat_average)/SMOOTH_FACTOR;
        lon_average_x= (lon_average_x*(SMOOTH_FACTOR-1)+lon_average)/SMOOTH_FACTOR;
        alt_average_x= (alt_average_x*(SMOOTH_FACTOR-1)+alt_average)/SMOOTH_FACTOR; }

      if (str_sec > 0) {
        nonzero_sec++; }
      last_last_imode= last_imode;
      if ((str_sec >= MAX_STR_SEC)||(nonzero_sec >= MAX_NONZERO_SEC)) {
        last_imode= true; }
      if (str_sec == 0) {
        nonzero_sec= 0;
        last_imode= false; }
      last_str_sec= str_sec;
      str_sec= 0;

      if (verbose_info_flag) {
        printf ("lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf, time precision: %+.0Lf nsec\n", lat_average_x, lon_average_x, alt_average_x, time_precision*1000000000.0l);
        fflush (stdout); }

      now_time= ensec_time ();
      if ((last_accuracy_ok)&&(last_pos_ok)&&(last_sec_ok)&&
          (now_time > last_transmition_time + 600000000000ll)) {
        last_channels= 0;
        last_values= 0;
        last_bits= nonzero_sec;
        strcpy (last_data, "-");
        send_strike (sock_id, serv_addr, username, password);
        last_transmition_time= now_time; }

      log_status(); } }

  // BLSIG sentence type 1
  else if ((sscanf (line, "$BLSIG,%6llx,%2x,%2x%c", &counter, &A, &B, &status) == 4)&&(status == '*')) {
    sprintf (last_data, "%02x%02x",A,B);
    last_channels= 2;
    last_values= 1;
    last_bits= 8;
    BLSIG_found= true; }

  // BLSIG sentence type 2
  else if ((sscanf (line, "$BLSIG,%6llx,%3x,%3x%c", &counter, &A, &B, &status) == 4)&&(status == '*')) {
    sprintf (last_data, "%03x%03x",A,B);
    last_channels= 2;
    last_values= 1;
    last_bits= 12;
    BLSIG_found= true; }

  // BLSIG sentence type 3
  else if ((sscanf (line, "$BLSEQ,%6llx,%256s%c", &counter, last_data, &status) == 3)&&(status == '*')) {
    last_channels= 2;
    last_values= 64;
    last_bits= 8;
    BLSIG_found= true; }

  // BD sentence type 1
  else if ((sscanf (line, "$BD,%6llx,%256s%c", &counter, last_data, &status) == 3)&&(status == '*')) {
    last_channels= 2;
    last_values= 64;
    last_bits= 8;
    BLSIG_found= true; }

  // BM sentence type 1
  else if ((sscanf (line, "$BM,%6llx,%256s%c", &counter, last_data, &status) == 3)&&(status == '*')) {
    last_channels= 1;
    last_values= 128;
    last_bits= 8;
    BLSIG_found= true; }

  // Firmware sentence
  else if (sscanf (line, "Firmware Version: %s", firmware) == 1) {
    strcpy(last_firmware, firmware);
    write_log_messages (line); }

  // unknown sentence
  else {
    sprintf (buf, "unknown sentence: %s", line);
    write_log_messages (buf); }

  if (BLSIG_found) { 
    if (counter >  last_counter) {
      dif= counter-last_counter; }
    else {
      dif= (counter+0x1000000ll)-last_counter; }

    last_nsec= (long long)(dif*RING_BUFFER_SIZE*1000000000ll)/dif_sum;
    str_sec++;

    // send only if precision is ok
    if ((last_accuracy_ok)&&(last_pos_ok)&&(last_sec_ok)&&(!(last_imode))) {
      send_strike (sock_id, serv_addr, username, password);
      last_transmition_time= now_time; } }
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

  char *logfile_log= 0;
  char *logfile_output= 0;
  char *logfile_sent= 0;
  char *serial_device_in= 0;
  char *serial_device_out= 0;
  bool serial_device_out_flag= false;
  char *username;
  char *password;
  verbose_log_flag= false;
  verbose_info_flag= false;
  verbose_output_flag= false;
  verbose_sent_flag= false;
  bool SBAS_flag= false;
  bool help_flag= false;

  bool flag_found;
  do {
    flag_found= false;
    if ((argc > 0) && ((strcmp (argv[0],"-L") == 0)||(strcmp (argv[0],"--syslog") == 0))) {
    flag_found= true;
      syslog_flag= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0], "-ll") == 0)|(strcmp (argv[0],"--log_log") == 0))) {
      flag_found= true;
      logfile_log_flag= true;
      logfile_log= argv[1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0], "-lo") == 0)|(strcmp (argv[0],"--log_output") == 0))) {
      flag_found= true;
      logfile_output_flag= true;
      logfile_output= argv[1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0], "-ls") == 0)|(strcmp (argv[0],"--log_sent") == 0))) {
      flag_found= true;
      logfile_sent_flag= true;
      logfile_sent= argv[1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0],"-vl") == 0)||(strcmp (argv[0],"--verbose_log") == 0))) {
      flag_found= true;
      verbose_log_flag= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0],"-vi") == 0)||(strcmp (argv[0],"--verbose_info") == 0))) {
      flag_found= true;
      verbose_info_flag= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0],"-vo") == 0)||(strcmp (argv[0],"--verbose_output") == 0))) {
      flag_found= true;
      verbose_output_flag= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0],"-vs") == 0)||(strcmp (argv[0],"--verbose_sent") == 0))) {
      flag_found= true;
      verbose_sent_flag= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0], "-e") == 0)||(strcmp (argv[0],"--echo") == 0))) {
      flag_found= true;
      serial_device_out_flag= true;
      serial_device_out= argv[1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && ((strcmp (argv[0], "-s") == 0)||(strcmp (argv[0],"--SBAS") == 0))) {
      flag_found= true;
      SBAS_flag= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && ((strcmp (argv[0],"-h") == 0)||(strcmp (argv[0], "--help") == 0))) {
      flag_found= true;
      help_flag= true;
      argc--;
      argv++; } }
  while (flag_found);

  if ((help_flag)||(argc != 5)) {
    printf ("%s: [-h] [-l] [-le file] [-lo file] [-ls file] [-ve] [-vi] [-vo] [-vs] [-e serial_device] [-s] gps_type baud_rate serial_device username password\n", program_name);
    printf ("gps_type         : gps type (SANAV, Garmin, or SiRF ('-' for no initialization)\n");
    printf ("baud_rate        : baud rate (4800, 9600, 19200, or 38400)\n");
    printf ("serial_device    : serial device (example: /dev/ttyS0)\n");
    printf ("username         : username (example: PeterPim) \n");
    printf ("password         : password (example: xxxxxxxx)\n");
    printf ("-L               : write log messages to the system message logger\n");
    printf ("                   altervative: --syslog \n");
    printf ("-ll file         : write log messages to file\n");
    printf ("                   alternative: --log_log file\n");
    printf ("-lo file         : write board output to file\n");
    printf ("                   alternative: --log_output file\n");
    printf ("-ls file         : write sent information to file\n");
    printf ("                   alternative: --log_sent file\n");
    printf ("-vl              : verbose mode, write log messages to stdout\n");
    printf ("                   alternative: --verbose_log\n");
    printf ("-vi              : verbose mode, write system information to stdout\n");
    printf ("                   alternative: --verbose_info\n");
    printf ("-vo              : verbose mode, write board output to stdout\n");
    printf ("                   alternative: --verbose_output\n");
    printf ("-vs              : verbose mode, write sent information to stdout\n");
    printf ("                   alternative: --verbose_sent\n");
    printf ("-h               : print this help text\n");
    printf ("                   alternative: --help\n");
    printf ("-e serial_device : serial device for input echo\n");
    printf ("                   alternative: --echo serial_device\n");
    printf ("-s               : activate SBAS (WAAS/EGNOS/MSAS) support\n");
    printf ("                   alternative: --SBAS\n");
    exit (-1); }

  gps_type= argv[0];
  baudrate= atoi (argv[1]);
  serial_device_in= argv[2];
  strcpy(last_firmware, "-");
  username= argv[3];
  password= argv[4];

  if (logfile_log_flag) {
    logfile_log_fd= fopen(logfile_log, "w"); }
  if (logfile_output_flag) {
    logfile_output_fd= fopen(logfile_output, "w"); }
  if (logfile_sent_flag) {
    logfile_sent_fd= fopen(logfile_sent, "w"); }

  char buf[STRING_BUFFER_SIZE];
		   
  write_log_messages ("tracker started\n");

  init_gps (serial_device_in, gps_type, baudrate, SBAS_flag);

  int f= open (serial_device_in, O_RDWR | O_NOCTTY );
  if (f < 0) {
    perror ("open ()");
    exit (-1); }
  set_baudrate (f, baudrate);

  int e= 0;
  if (serial_device_out_flag) {
    e= open (serial_device_out, O_RDWR | O_NOCTTY );
    if (e < 0) {
      perror ("open ()");
      exit (-1); }
    set_baudrate (e, baudrate); }

  int sock_id= socket (AF_INET, SOCK_DGRAM, 0);
  if (sock_id == -1) {
    perror ("socket ()");
    exit (-1); }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family= AF_INET;
  serv_addr.sin_port= htons (SERVER_PORT);
  serv_addr.sin_addr.s_addr= inet_addr (SERVER_ADDR);

  if (serv_addr.sin_addr.s_addr == INADDR_NONE) {
    /* host not given by IP but by name */
    struct hostent *host_info= gethostbyname (SERVER_ADDR);
    if (host_info == NULL) {
      write_log_messages ("gethostbyname () failed, try again in 10 seconds\n");
      sleep (10);
      hostent *host_info= gethostbyname (SERVER_ADDR);
      if (host_info == NULL) {
        write_log_messages ("gethostbyname () failed, try again in 60 seconds\n");
        sleep (10);
        hostent *host_info= gethostbyname (SERVER_ADDR);
        if (host_info == NULL) {
          write_log_messages ("gethostbyname () failed, give up, check internet connection\n");
          perror ("gethostbyname ()");
          close (sock_id);
          exit (-1); } } }
     memcpy((char*) &serv_addr.sin_addr.s_addr, host_info->h_addr, host_info->h_length); }

  char c;
  int i=0;
  while (true) {
    if (read (f, &c, 1) == 1) {
      if (serial_device_out_flag) {
        write (e, &c, 1); }
      if (verbose_output_flag) {
        putchar (c); }
      if (logfile_output_flag) {
        fwrite (&c, 1, 1, logfile_output_fd);
        fflush (logfile_log_fd); }
      buf [i]= c;
      if (i < STRING_BUFFER_SIZE-2) {
        i++; }
      if (c == '\n') {
        buf [i]= 0;
        evaluate (buf, sock_id, (struct sockaddr *)&serv_addr, username, password);
        i= 0; } } }
}

