/****************************************************************
 *
 * Copyright (c) 2011
 * All rights reserved.
 *
 * Hochschule Bonn-Rhein-Sieg
 * University of Applied Sciences
 * Computer Science Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author:
 * Jan Paulus, Nico Hochgeschwender, Michael Reckhaus, Azamat Shakhimardanov
 * Supervised by:
 * Gerhard K. Kraetzschmar
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This sofware is published under a dual-license: GNU Lesser General Public 
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ****************************************************************/
#include "youbot/YouBotGripperBar.hpp"
namespace youbot {

YouBotGripperBar::YouBotGripperBar(const unsigned int barNo, const unsigned int jointNo, const std::string& configFilePath) {
  // Bouml preserved body begin 000E0371
    this->jointNumber = jointNo;
    this->mailboxMsgRetries = 200;
    this->timeTillNextMailboxUpdate = 1; //ms
    this->barNo = barNo;

    ethercatMaster = &(EthercatMaster::getInstance("youbot-ethercat.cfg", configFilePath));
  // Bouml preserved body end 000E0371
}

YouBotGripperBar::~YouBotGripperBar() {
  // Bouml preserved body begin 000E03F1
  // Bouml preserved body end 000E03F1
}

void YouBotGripperBar::getConfigurationParameter(YouBotGripperParameter& parameter) {
  // Bouml preserved body begin 000E05F1
  
  if (parameter.getType() == MOTOR_CONTOLLER_PARAMETER) {

      YouBotSlaveMailboxMsg message;
      parameter.getYouBotMailboxMsg(message);
      message.stctOutput.commandNumber = GAP;
      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.motorNumber = this->barNo;
      message.parameterName = parameter.getName();
      
      if (retrieveValueFromMotorContoller(message)) {
        parameter.setYouBotMailboxMsg(message);
      } else {
        throw JointParameterException("Unable to get parameter: " + parameter.getName() + " from the gripper");
      }
    }else{
      throw JointParameterException("Parameter " + parameter.getName() + " is not a motor controller parameter of the gripper");
    }
  // Bouml preserved body end 000E05F1
}

void YouBotGripperBar::setConfigurationParameter(const YouBotGripperParameter& parameter) {
  // Bouml preserved body begin 000E0671
   if (parameter.getType() == MOTOR_CONTOLLER_PARAMETER) {

      YouBotSlaveMailboxMsg message;
      parameter.getYouBotMailboxMsg(message);
      message.stctOutput.commandNumber = SAP;
      message.stctOutput.moduleAddress = GRIPPER;
      message.stctOutput.motorNumber = this->barNo;
      message.parameterName = parameter.getName();
      
      if (!setValueToMotorContoller(message)) {
        throw JointParameterException("Unable to set parameter: " + parameter.getName() + " to the gripper");
      }
    }else{
      throw JointParameterException("Parameter " + parameter.getName() + " is not a motor controller parameter of the gripper");
    }
  // Bouml preserved body end 000E0671
}

void YouBotGripperBar::getConfigurationParameter(YouBotSlaveMailboxMsg& parameter) {
  // Bouml preserved body begin 000E0A71
  if (!retrieveValueFromMotorContoller(parameter)) {
     throw JointParameterException("Unable to get parameter from the gripper");
   }
   this->parseMailboxStatusFlags(parameter);
  // Bouml preserved body end 000E0A71
}

void YouBotGripperBar::setData(const GripperBarEncoterSetpoint& encoderSetpoint) {
  // Bouml preserved body begin 000E0CF1
    YouBotSlaveMailboxMsg message;
    message.stctOutput.moduleAddress = GRIPPER;
    message.stctOutput.commandNumber = MVP;
    message.stctOutput.typeNumber = 0; //move gripper absolute
    message.stctOutput.motorNumber = this->barNo;
    message.stctOutput.value = encoderSetpoint.barEncoder * -1;

    setValueToMotorContoller(message);
    
  // Bouml preserved body end 000E0CF1
}

void YouBotGripperBar::getData(GripperSensedVelocity& barVelocity) {
  // Bouml preserved body begin 000E0DF1
   YouBotSlaveMailboxMsg message;
    message.stctOutput.moduleAddress = GRIPPER;
    message.stctOutput.commandNumber = GAP; 
    message.stctOutput.typeNumber = 3; //actual velocity
    message.stctOutput.value = 0;
    
    message.stctOutput.motorNumber = this->barNo;

    retrieveValueFromMotorContoller(message);
    //std::cout << message.stctInput.value << std::endl;
    
    barVelocity.barVelocity = message.stctInput.value;

    
  // Bouml preserved body end 000E0DF1
}

void YouBotGripperBar::parseMailboxStatusFlags(const YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 000E0E71
    std::stringstream errorMessageStream;
    errorMessageStream << "Joint " << this->jointNumber << ": ";
    std::string errorMessage;
    errorMessage = errorMessageStream.str();


    switch(mailboxMsg.stctInput.status){
      case NO_ERROR:
        break;
      case INVALID_COMMAND:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << "; Command no: " << mailboxMsg.stctOutput.commandNumber << " is an invalid command!" ;
      //    throw JointParameterException(errorMessage + "invalid command");
        break;
      case WRONG_TYPE:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << " has a wrong type!";
      //    throw JointParameterException(errorMessage + "wrong type");
        break;
      case INVALID_VALUE:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << " Value: " << mailboxMsg.stctOutput.value << " is a invalid value!";
      //    throw JointParameterException(errorMessage + "invalid value");
        break;
      case CONFIGURATION_EEPROM_LOCKED:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << " Configuration EEPROM locked";
      //    throw JointParameterException(errorMessage + "configuration EEPROM locked");
        break;
      case COMMAND_NOT_AVAILABLE:
        LOG(error) << errorMessage << "Parameter name: " << mailboxMsg.parameterName << "; Command no: " << mailboxMsg.stctOutput.commandNumber << "Command is not available!";
      //    throw JointParameterException(errorMessage + "command not available");
        break;
    }
   

  // Bouml preserved body end 000E0E71
}

bool YouBotGripperBar::setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg) {
  // Bouml preserved body begin 000E0EF1

    YouBotSlaveMailboxMsg mailboxMsgBuffer;
    mailboxMsgBuffer = mailboxMsg;
    bool unvalid = true;
    unsigned int retry = 0;

    ethercatMaster->setMailboxMsgBuffer(mailboxMsgBuffer, this->jointNumber);
//    LOG(trace) << "set Output CommandNumber " << (int) mailboxMsgBuffer.stctOutput.commandNumber
//                  << " moduleAddress " << (int) mailboxMsgBuffer.stctOutput.moduleAddress
//                  << " motorNumber " << (int) mailboxMsgBuffer.stctOutput.motorNumber
//                  << " typeNumber " << (int) mailboxMsgBuffer.stctOutput.typeNumber
//                  << " value " << mailboxMsgBuffer.stctOutput.value;

    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {
          
       
      if (ethercatMaster->getMailboxMsgBuffer(mailboxMsgBuffer, this->jointNumber) &&
          mailboxMsgBuffer.stctInput.status == NO_ERROR) {
        unvalid = false;
      } else {
        SLEEP_MILLISEC(timeTillNextMailboxUpdate);
        retry++;
      }
//      LOG(trace) << "set Input CommandNumber " << (int) mailboxMsgBuffer.stctInput.commandNumber
//                  << " moduleAddress " << (int) mailboxMsgBuffer.stctInput.moduleAddress
//                  << " replyAddress " << (int) mailboxMsgBuffer.stctInput.replyAddress
//                  << " status " << (int) mailboxMsgBuffer.stctInput.status
//                  << " value " << mailboxMsgBuffer.stctInput.value;
    } while (retry < mailboxMsgRetries && unvalid);

    if (unvalid) {
      this->parseMailboxStatusFlags(mailboxMsgBuffer);
      return false;
    } else {
      return true;
    }

  // Bouml preserved body end 000E0EF1
}

bool YouBotGripperBar::retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message) {
  // Bouml preserved body begin 000E0F71

    bool unvalid = true;
    unsigned int retry = 0;

    ethercatMaster->setMailboxMsgBuffer(message, this->jointNumber);
//     LOG(trace) << "get Output CommandNumber " << (int) message.stctOutput.commandNumber
//                  << " moduleAddress " << (int) message.stctOutput.moduleAddress
//                  << " motorNumber " << (int) message.stctOutput.motorNumber
//                  << " typeNumber " << (int) message.stctOutput.typeNumber
//                  << " value " << message.stctOutput.value
//                  << " No " << this->jointNumber;

    SLEEP_MILLISEC(timeTillNextMailboxUpdate);

    do {
         
       
      if (ethercatMaster->getMailboxMsgBuffer(message, this->jointNumber) &&
          message.stctInput.status == NO_ERROR) {
        unvalid = false;
      } else {
        SLEEP_MILLISEC(timeTillNextMailboxUpdate);
        retry++;
      }
//      LOG(trace) << "get input CommandNumber " << (int) message.stctInput.commandNumber
//                 << " moduleAddress " << (int) message.stctInput.moduleAddress
//                 << " replyAddress " << (int) message.stctInput.replyAddress
//                 << " status " << (int) message.stctInput.status
//                 << " value " << message.stctInput.value
//                 << " No " << this->jointNumber;
         
    } while (retry < mailboxMsgRetries && unvalid);

    if (unvalid) {
      this->parseMailboxStatusFlags(message);
      return false;
    } else {
      return true;
    }

  // Bouml preserved body end 000E0F71
}


} // namespace youbot