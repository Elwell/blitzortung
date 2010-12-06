/*
    Copyright (C) 2003-2010  Egon Wanke <blitzortung@gmx.org> 1

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
#include <math.h>

long double fabsl (long double x); // this is only necessary for the OpenWrt
                                   // but do not disturb other systems

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#define VERSION                 "LT&nbsp;Ver.&nbsp;19" // version string send to server
#define SERVER_ADDR             "rechenserver.de"      // server address
#define SERVER_PORT             8308                   // server port
#define STRING_BUFFER_SIZE      2048                   // maximal buffer size for the strings we use
#define RING_BUFFER_SIZE        20                     // ring buffer size for averaged computation of counter difference
#define SMOOTH_FACTOR           3600                   //
#define POS_PRECISION           0.001000l              // position precision in degree to reach before sending data
#define TIME_PRECISION          0.000001l              // time precision to reach before sending data

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

// if verbose flag is true, the program outputs helpful information on standard output
bool verbose_flag;
 
// if logfile flag is true, the program outputs helpful information in a logfile
bool logfile_flag= false;
FILE *log_fd;

// all these global last variables store the information of the last BLSEC sentence
long long last_counter= -1;
int last_year, last_mon, last_day, last_hour, last_min, last_sec, last_last_sec, last_sat;
int last_channels;
int last_values;
int last_bits;
int last_nsec_lag;
long double last_lat, last_lon, last_alt;
long long last_nsec;
char last_status= '-', last_last_status;
bool last_pos_ok= false, last_time_ok= false;
bool last_com_ok= false, last_last_com_ok= false;
bool last_last_time_ok= false, last_last_pos_ok= false;
char last_data[STRING_BUFFER_SIZE];
char last_Firmware_Version[STRING_BUFFER_SIZE];
long long last_transmition_time= 0ll;

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

// the ring buffer is used to compute the averred position and time difference between two BLSEC sentences
struct ring_buffer_type {
  long long dif;
  long double lat;
  long double lon;
  long double alt; } ring_buffer[RING_BUFFER_SIZE];

/******************************************************************************/
/***** initialization string for the gps devices ******************************/
/******************************************************************************/

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
/***** write to log file ******************************************************/
/******************************************************************************/

void write_to_log (const char *text)
{
  int year, mon, day, min, hour;
  long double sec_nsec;
  char buf [STRING_BUFFER_SIZE];

  ensec_to_utc_ctime (ensec_time(), &year, &mon, &day, &hour, &min, &sec_nsec);
  sprintf (buf, "%04d-%02d-%02d %02d:%02d:%02.0Lf, PID: %d,", year, mon, day, hour, min, sec_nsec, (int)getpid());

  fprintf (log_fd, "%s %s", buf, text);
  fflush (log_fd);
  if (verbose_flag) {
    printf ("%s %s", buf, text);
    fflush (stdout); }
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

// set baudrate
void set_baudrate (int f, int baudrate)
{
  struct termios tio;
  tio.c_iflag= IGNBRK | IGNPAR ;
  tio.c_oflag= OPOST | ONLCR ;
  if (baudrate == 4800) {
    tio.c_cflag= B4800 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 9600) {
    tio.c_cflag= B9600 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 19200) {
    tio.c_cflag= B19200 | CS8 | CLOCAL | CREAD ; }
  else if (baudrate == 38400) {
    tio.c_cflag= B38400 | CS8 | CLOCAL | CREAD ; }
  else {
    printf ("Do not know how to initialize the tty with %d baud!\n", baudrate);
    return; }
  tio.c_lflag= 0;
  tio.c_cc[VTIME]= 0;
  tio.c_cc[VMIN]= 1;
  tcsetattr (f, TCSANOW, &tio);
}

// initialize the GPS modul
void init_gps (const char *serial_device, char *gps_type, int baudrate)
{
  char init_string [STRING_BUFFER_SIZE];

  if ((strcmp (gps_type, "sjn") == 0) && (baudrate == 4800)) {
    strcpy (init_string, init_gps_sjn); }

  else if ((strcmp (gps_type, "garmin") == 0) && (baudrate == 4800)) {
    strcpy (init_string, init_gps_garmin_4800); }
  else if ((strcmp (gps_type, "garmin") == 0) && (baudrate == 9600)) {
    strcpy (init_string, init_gps_garmin_9600); }
  else if ((strcmp (gps_type, "garmin") == 0) && (baudrate == 19200)) {
    strcpy (init_string, init_gps_garmin_19200); }

  else if ((strcmp (gps_type, "sirf") == 0) && (baudrate == 4800)) {
    strcpy (init_string, init_gps_sirf_4800); }
  else if ((strcmp (gps_type, "sirf") == 0) && (baudrate == 9600)) {
    strcpy (init_string, init_gps_sirf_9600); }
  else if ((strcmp (gps_type, "sirf") == 0) && (baudrate == 19200)) {
    strcpy (init_string, init_gps_sirf_19200); }
  else if ((strcmp (gps_type, "sirf") == 0) && (baudrate == 38400)) {
    strcpy (init_string, init_gps_sirf_38400); }
  else {
    if (strcmp (gps_type, "-") != 0) {
      printf ("Do not know how to initialize GPS device %s with %d baud!\n", gps_type, baudrate); }
    return; }
    
  fill_checksum (init_string);
  if (verbose_flag) {
    printf ("%s",init_string);
    fflush (stdout); }

  int br= 4800;
  for (int b=0; b<4; b++) {
    int f= open (serial_device, O_RDWR | O_NOCTTY );
    if (f == -1) {
      perror ("open()");
      exit (-1); }
    set_baudrate (f, br);
    br+= br;
    write (f, "55555\n55555\n55555\n", 18); // this is only for synchronization
    write (f, init_string, strlen(init_string));
    close (f); }
}

/******************************************************************************/
/***** send_strike ************************************************************/
/******************************************************************************/

void send_strike (int sock_id, struct sockaddr *serv_addr, const char *username, const char *password)
{
  char buf [STRING_BUFFER_SIZE];
  sprintf (buf, "%04d-%02d-%02d %02d:%02d:%02d.%09lld %.6Lf %.6Lf %.0Lf %s %s %c %d %d %d %d %s %s\n",
    last_year, last_mon, last_day, last_hour, last_min, last_sec, last_nsec, lat_average_x, lon_average_x, alt_average_x, username, password, last_status, last_channels, last_values, last_bits, last_nsec_lag, last_data, VERSION);

  if (sendto (sock_id, buf, strlen (buf), 0, serv_addr, sizeof (sockaddr)) == -1) {
    write_to_log ("error: sendto ()\n");
    return; }
  if (verbose_flag) { 
    printf ("%s", buf);
    fflush (stdout); }
}

/******************************************************************************/
/***** evaluate ***************************************************************/
/******************************************************************************/

void log_precision ()
{
  char buf [STRING_BUFFER_SIZE];

  if ((!(last_last_com_ok))&&(last_com_ok)) {
    write_to_log ("GPS data sentences ok\n"); }
  if ((last_last_com_ok)&&(!(last_com_ok))) {
    write_to_log ("GPS data sentences lost\n"); }

  if (last_status != last_last_status) {
    sprintf (buf, "GPS status changed to '%c'\n", last_status);
    write_to_log (buf); }

  if ((!(last_last_pos_ok))&&(last_pos_ok)) {
    sprintf (buf, "Position stable, lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf\n", lat_average, lon_average, alt_average);
    write_to_log (buf); }
  if ((last_last_pos_ok)&&(!(last_pos_ok))) {
    sprintf (buf, "Position lost, lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf\n", last_lat, last_lon, last_alt);
    write_to_log (buf); }

  if ((!(last_last_time_ok))&&(last_time_ok)) {
    sprintf (buf, "1PPS signal ok, %+.0Lf nsec\n", time_precision*1000000000.0l);
    write_to_log (buf); }
  if ((last_last_time_ok)&&(!(last_time_ok))) {
    sprintf (buf, "1PPS signal inaccurate, %+.0LF nsec\n", time_precision*1000000000.0l);
    write_to_log (buf); }
}

void evaluate (const char *line, int sock_id, struct sockaddr *serv_addr, const char *username, const char *password)
{
  int year, mon, day, min, hour, sec= 0, lat_deg, lon_deg, sat= 0, A, B;
  long double lat, lat_min, lon, lon_min, alt= 0.0l;
  char ns, we, status;
  long long counter, dif;
  bool BLSIG_found= false;
  char buf[STRING_BUFFER_SIZE];

  // BLSEC sentences
  if ((sscanf (line, "$BLSEC,%llx,%c,%2d%2d%2d,%2d%2d%2d,%2d%Lf,%c,%3d%Lf,%c,%Lf,M,%d*",
      &counter, &status, &hour, &min, &sec, &day, &mon, &year, &lat_deg, &lat_min, &ns, &lon_deg, &lon_min, &we, &alt, &sat) == 16) ||
      (sscanf (line, "$BLSEC,%2d%2d%2d,%2d%2d%2d,%c,%2d%Lf,%c,%3d%Lf,%c,%llx*",
      &hour, &min, &sec, &day, &mon, &year, &status, &lat_deg, &lat_min, &ns, &lon_deg, &lon_min, &we, &counter) == 14)) {

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
      last_last_com_ok= last_com_ok;
      last_com_ok= (sec == (last_sec+1)%60);
      last_sec= sec;
      last_last_status= last_status;
      last_status= status;
      last_sat= sat;
      last_lat= lat;
      last_lon= lon;
      last_alt= alt;
      if (counter >  last_counter) {
        dif= counter-last_counter; }
      else {
        dif= (counter+0x1000000ll)-last_counter; }
      last_counter= counter;

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

      last_last_time_ok= last_time_ok;
      last_time_ok= (fabsl (time_precision) < TIME_PRECISION);

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

      if (verbose_flag) {
        printf ("lat: %+.6Lf, lon: %+.6Lf, alt: %+.1Lf, time precision: %+.0Lf nsec\n", lat_average_x, lon_average_x, alt_average_x, time_precision*1000000000.0l);
        fflush (stdout); }

      now_time= ensec_time ();
      if ((last_time_ok)&&(last_pos_ok)&&(last_com_ok)&&
          (now_time > last_transmition_time + 600000000000ll)) {
        last_channels= 0;
        last_values= 0;
        last_bits= 0;
        last_nsec_lag= 0;
        strcpy (last_data, "-");
        last_transmition_time= now_time;
        send_strike (sock_id, serv_addr, username, password); }

      if (logfile_flag) {
        log_precision(); } } }

  // BLSIG sentence type 1
  else if ((sscanf (line, "$BLSIG,%6llx,%2x,%2x%c", &counter, &A, &B, &status) == 4)&&(status == '*')) {
    sprintf (last_data, "%02x%02x",A,B);
    last_channels= 2;
    last_values= 1;
    last_bits= 8;
    last_nsec_lag= 0;
    BLSIG_found= true; }

  // BLSIG sentence type 2
  else if ((sscanf (line, "$BLSIG,%6llx,%3x,%3x%c", &counter, &A, &B, &status) == 4)&&(status == '*')) {
    sprintf (last_data, "%03x%03x",A,B);
    last_channels= 2;
    last_values= 1;
    last_bits= 10;
    last_nsec_lag= 0;
    BLSIG_found= true; }

  // BLSIG sentence type 3
  else if ((sscanf (line, "$BLSEQ,%6llx,%256s%c", &counter, last_data, &status) == 3)&&(status == '*')) {
    last_channels= 2;
    last_values= 64;
    last_bits= 8;
    last_nsec_lag= 3125;
    BLSIG_found= true; }

  // BLSTR sentence type 1
  else if ((sscanf (line, "$BLSTR,%6llx,%256s%c", &counter, last_data, &status) == 3)&&(status == '*')) {
    last_channels= 1;
    last_values= 128;
    last_bits= 8;
    last_nsec_lag= 2930;
    BLSIG_found= true; }

  // BLSTR sentence type 2
  else if ((sscanf (line, "$BLSTR,%6llx,%512s%c", &counter, last_data, &status) == 3)&&(status == '*')) {
    last_channels= 1;
    last_values= 256;
    last_bits= 8;
    last_nsec_lag= 2930;
    BLSIG_found= true; }

  // Firmware sentence
  else if (sscanf (line, "Firmware %s", last_Firmware_Version) == 1) {
    if (logfile_flag) {
      write_to_log (line); } }

  // unknown sentence
  else if (logfile_flag) {
    if (logfile_flag) {
      sprintf (buf, "unknown sentence: %s", line);
      write_to_log (buf); } }

  if (BLSIG_found) { 
    if (counter >  last_counter) {
      dif= counter-last_counter; }
    else {
      dif= (counter+0x1000000ll)-last_counter; }

    last_nsec= (long long)(dif*RING_BUFFER_SIZE*1000000000ll)/dif_sum;

    if (logfile_flag) {
      log_precision(); }

    // send only if precision is ok
    if ((last_time_ok)&&(last_pos_ok)&&(last_com_ok)) {
      last_transmition_time= now_time;
      send_strike (sock_id, serv_addr, username, password); } }
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

  char *logfile= 0;
  char *gps_type;
  int baudrate;
  char *serial_device;
  char *username;
  char *password;
  verbose_flag= false;
  bool help_flag= false;

  bool flag_found;
  do {
    flag_found= false;
    if ((argc > 0) && (strcmp (argv[0],"-l") == 0)) {
      flag_found= true;
      logfile_flag= true;
      logfile= argv[1];
      argc-=2;
      argv+=2; }
    if ((argc > 0) && (strcmp (argv[0],"-v") == 0)) {
      flag_found= true;
      verbose_flag= true;
      argc-=1;
      argv+=1; }
    if ((argc > 0) && (strcmp (argv[0],"-h") == 0)) {
      flag_found= true;
      help_flag= true;
      argc--;
      argv++; } }
  while (flag_found);

  if ((help_flag)||(argc != 5)) {
    printf ("%s: [-v] [-h] [-l logfile] gps_type baudrate serial_device username password\n",program_name);
    printf ("gps_type      : gps type (sjn, garmin, or sirf ('-' for no initialization)\n");
    printf ("baudrate      : baudrate (4800, 9600, 19200, or 38400)\n");
    printf ("serial_device : serial device (example: /dev/ttyS0)\n");
    printf ("username      : username (example: PeterPim) \n");
    printf ("password      : password (example: xxxxxxxx)\n");
    printf ("-l logfile    : log tracker information\n");
    printf ("-v            : verbose mode\n");
    printf ("-h            : print this help text\n");
    exit (-1); }

  gps_type= argv[0];
  baudrate= atoi (argv[1]);
  serial_device= argv[2];
  username= argv[3];
  password= argv[4];

  init_gps (serial_device, gps_type, baudrate);

  int f= open (serial_device, O_RDWR | O_NOCTTY );
  if (f < 0) {
    perror ("open ()");
    exit (-1); }
  set_baudrate (f, baudrate);

  if (logfile_flag) {
    log_fd= fopen(logfile, "w");
    write_to_log ("tracker started\n"); }

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
      write_to_log ("gethostbyname () failed, try again in 10 seconds\n");
      sleep (10);
      hostent *host_info= gethostbyname (SERVER_ADDR);
      if (host_info == NULL) {
        write_to_log ("gethostbyname () failed, try again in 60 seconds\n");
        sleep (10);
        hostent *host_info= gethostbyname (SERVER_ADDR);
        if (host_info == NULL) {
          write_to_log ("gethostbyname () failed, give up, check internet connection\n");
          perror ("gethostbyname ()");
          close (sock_id);
          exit (-1); } } }
     memcpy((char*) &serv_addr.sin_addr.s_addr, host_info->h_addr, host_info->h_length); }

  char c;
  int i=0;
  char buf[STRING_BUFFER_SIZE];
  while (true) {
    if (read (f, &c, 1) == 1) {
      buf [i]= c;
      if (verbose_flag) {
        putchar (c); }
      if (i < STRING_BUFFER_SIZE-2) {
        i++; }
      if (c == '\n') {
        buf [i]= 0;
        evaluate (buf, sock_id, (struct sockaddr *)&serv_addr, username, password);
        i= 0; } } }
}
