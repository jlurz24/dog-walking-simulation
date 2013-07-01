/**
 * Driver to test linking plugin
 */
#include <stdio.h>
#include "leash_model_plugin.cpp"

int main(int argc, const char* argv[]){
   gazebo::LeashModelPlugin* plugin = new gazebo::LeashModelPlugin();
   delete plugin;  
}
