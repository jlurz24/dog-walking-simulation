/**
 * Driver to test linking plugin
 */
#include <stdio.h>
#include "leash_world_plugin.cpp"

int main(int argc, const char* argv[]){
   gazebo::LeashWorldPlugin* plugin = new gazebo::LeashWorldPlugin();
   delete plugin;  
}
