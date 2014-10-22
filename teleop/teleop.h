//Various functions shared by the teleop programs

#ifndef HX_TELEOP_H
#define HX_TELEOP_H

#include <haptix/comm/haptix.h>

const hxAPLMotors mcp_indices[5] = {motor_little_mcp, motor_ring_mcp, motor_middle_mcp, motor_index_mcp, motor_thumb_mcp};

void coupling_v1(_hxCommand* cmd){
    //Version 1 Joint Coupling modeling
    
    //this relies on the ordering of enums, bit messy
    for(unsigned int k = 0; k < 5; k++){
      //Check if slider was changeded
      
      hxAPLMotors mcp = mcp_indices[k];
      
      float mcp_commanded = cmd->ref_pos[mcp];
      if(mcp_commanded <= 0){
        if(mcp == motor_thumb_mcp){
          cmd->ref_pos[mcp+1] = 0; //thumb dip
        } else {
          cmd->ref_pos[mcp+1] = 0; //pip
          cmd->ref_pos[mcp-1] = 0; //dip
        }
      } else {
        if(mcp == motor_thumb_mcp){
          cmd->ref_pos[mcp+1] = 8/9.0*mcp_commanded; //thumb dip
        } else {
          cmd->ref_pos[mcp+1] = 10/9.0*mcp_commanded; //pip
          cmd->ref_pos[mcp-1] = 8/9.0*mcp_commanded; //dip
        }
      }
    }

}

#endif
