#include "messages/state/differential_base.hpp"
#include "packages/engine_gems/state/io.hpp"
#include <termio.h>
#include <stdio.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include "drive_segway_sample.hpp"

namespace isaac {
#define PRINTHELP		  'h'
#define ADDLINEVEL		  'w'
#define DECLINEVEL		  's'
#define ADDANGULARVEL	  'a'
#define DECANGULARVEL	  'd'
#define PRINTCURVEL		  'f'
#define VELRESETZERO	  'g'
#define ENABLECMD         'e'
#define CHASSISPAUSE      'q'
#define NOMEANINGIN       0

static char const* print_help() {
    char const* printHelp = 
        "\t h : Displays the required keys and their meaning\n"
        "\t w : Increase forward speed by 0.1m/s\n"
        "\t s : Decrease forward speed by 0.1m/s\n"
        "\t a : Increase the angular velocity by 0.1rad/s\n"
        "\t d : Decrease the angular velocity by 0.1rad/s\n"
        "\t f : Displays current speed Settings\n"
        "\t g : Speed reset to zero\n"
        "\t e : Chassis enable switch\n"
        "\t q : Running pause. Click 'q'key again to resume running by the previous speed. W/S/A/D keys can also restore chassis running\n"
        "\t others : Do nothing\n";
    return printHelp;
}

 void DriveSegway::start() {
     enable_flag = 0;
     pause_flag = 0;
     set_line_vel = 0;
     set_angular_vel = 0;
     enable_switch = false;
     segway_init_ok = false;
     tickPeriodically();
 }

 void DriveSegway::tick() {
    if (!segway_init_ok)
    {
        if (!rx_segway_init_success().available()) return;
        auto proto = rx_segway_init_success().getProto();
        bool init_ok = proto.getFlag();
        if (init_ok != true) {
            printf("segway_rmplite init failed!\n");
            return;            
        }
        segway_init_ok = true;
        printf("%s\n",print_help() );
    }

    char keyvalue = get_keyboard();

    auto proto = tx_drive_enable_cmd().initProto();
    switch(keyvalue)
    {
        case ADDLINEVEL	:
            line_vel_cmd = ((set_line_vel) > 3 ? 3 : (set_line_vel));
            angular_vel_cmd = 0;
            break;
        case DECLINEVEL	:
            line_vel_cmd = ((set_line_vel) < -2 ? -2 : (set_line_vel));
            angular_vel_cmd = 0;
            break;
        case ADDANGULARVEL	:
            line_vel_cmd = 0;
            angular_vel_cmd = ((set_angular_vel) > 3 ? 3 : (set_angular_vel));
            break;
        case DECANGULARVEL	:
            line_vel_cmd = 0;
            angular_vel_cmd = ((set_angular_vel) < -3 ? -3 : (set_angular_vel));
            break;
        case ENABLECMD:
            enable_flag = ~enable_flag;
            enable_switch = ((enable_flag != 0) ? true : false);   
            std::cout << "enable_switch switching: " << std::boolalpha << enable_switch << std::endl;   
            if (!enable_switch) {
                set_line_vel = 0;
                set_angular_vel = 0;
                line_vel_cmd = 0;
                angular_vel_cmd = 0;
            }
            proto.setFlag(enable_switch);
            tx_drive_enable_cmd().publish();
            break;
        case VELRESETZERO :
            line_vel_cmd = set_line_vel;
            angular_vel_cmd = set_angular_vel;
            break;
        case CHASSISPAUSE :
            pause_flag = ~pause_flag;
            line_vel_cmd = (pause_flag != 0) ? 0 : set_line_vel;
            angular_vel_cmd = (pause_flag != 0) ? 0 : set_angular_vel;
            break;
        default:
            //do nothing
            break;
    }
    messages::DifferentialBaseControl speed_cmd;
    speed_cmd.linear_speed() = line_vel_cmd;
    speed_cmd.angular_speed() = angular_vel_cmd;
    ToProto(speed_cmd, tx_speed_cmd().initProto(), tx_speed_cmd().buffers());
    tx_speed_cmd().publish();
 }

static void changemode(int dir)
{
  static struct termios oldt, newt;
 
  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

static int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;
 
  tv.tv_sec = 0;
  tv.tv_usec = 0;
 
  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);
 
  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);
}

static int scanKeyboard()
{
    int input_char = NOMEANINGIN;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);

    changemode(1);
    if (kbhit()) {
        input_char = getchar();
        printf("\n");
    }
    changemode(0);

    tcsetattr(0,TCSANOW,&stored_settings);
    
    return input_char;
}

char  DriveSegway::get_keyboard() {
    char keyvalue = scanKeyboard();//getchar();
    
    switch (keyvalue) 
    {
        case PRINTHELP:
            printf("%s\n",print_help() );
            break;
        case ENABLECMD:
        case NOMEANINGIN :
            break;
        case CHASSISPAUSE :
            printf("Running pause. Click 'q'key again to resume running by the previous speed. W/S/A/D keys can also restore chassis running\n");
            break;
        case ADDLINEVEL :
            set_line_vel = ((set_line_vel < 3.0) ? (set_line_vel + 0.1) : set_line_vel);
            printf("The current linear set_velocity is %lf [m/s]\n", set_line_vel);
            if (set_line_vel >= 3.0) printf("The current linear set_velocity has reached its maximum value\n");
            break;
        case DECLINEVEL :
            set_line_vel = ((set_line_vel > -2.0) ? (set_line_vel - 0.1) : set_line_vel);
            printf("The current linear set_velocity is %lf [m/s]\n", set_line_vel);
            if (set_line_vel <= -2.0) printf("The current linear set_velocity has reached its minimum value\n");
            break;
        case ADDANGULARVEL :
            set_angular_vel = ((set_angular_vel < 3.0) ? (set_angular_vel + 0.1) : set_angular_vel);
            printf("The current angular  set_velocity is %lf [rad/s]\n", set_angular_vel);
            if (set_angular_vel >= 3.0) printf("The current angular set_velocity has reached its maximum value\n");
            break;
        case DECANGULARVEL :
            set_angular_vel = ((set_angular_vel > -3.0) ? (set_angular_vel - 0.1) : set_angular_vel);
            printf("The current angular set_velocity is %lf [rad/s]\n", set_angular_vel);
            if (set_angular_vel <= -3.0) printf("The current angular set_velocity has reached its minimum value\n");
            break;
        case PRINTCURVEL :
            printf("The current linear set_velocity is %lf [m/s]\n", set_line_vel);
            printf("The current angular  set_velocity is %lf [rad/s]\n", set_angular_vel);
            break;
        case VELRESETZERO :
            set_line_vel = 0;
            set_angular_vel = 0;
            printf("The current linear set_velocity is set to %lf [m/s]\n", set_line_vel);
            printf("The current angular  set_velocity is set to %lf [rad/s]\n", set_angular_vel);
            break;
        default :
            printf("Illegal input value ascii:%d, character:%c\n", keyvalue, keyvalue);
            break;
    }
    return keyvalue;
}


} // namespace isaac