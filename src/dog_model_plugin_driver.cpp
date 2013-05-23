/**
 * Driver to test linking plugin
 */
#include <stdio.h>
#include "dog_model_plugin.cpp"

int main(int argc, const char* argv[]){
   gazebo::DogModelPlugin* plugin = new gazebo::DogModelPlugin();
   delete plugin;  
}
