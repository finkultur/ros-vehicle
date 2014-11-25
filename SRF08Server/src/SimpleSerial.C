/*
Copyright (c) 2014, Thomas Petig <thomas@petig.eu>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include "SimpleSerial.H"

using namespace std;
using namespace boost;

int main(int argc, char* argv[])
{
   try {
      SimpleSerial serial("/dev/ttyUSB0", 19200);

      uint8_t cmd[5] = {0x5A, 0x01, 0x00, 0x00};
      serial.writeString(cmd, 4);
      uint32_t res = serial.readChar();
      cout<< "Firmware Version: " << std::hex << res <<endl;

      uint8_t start = 0xE0;
      cmd[0] = 0x55;
      cmd[1] = 0xE1;
      cmd[2] = 0x00;
      cmd[3] = 0x01;
      serial.writeString(cmd, 4);
       res = serial.readChar();
      cout<< "ret: " << std::hex << res <<endl;

      cmd[0] = 0x55;
      cmd[1] = 0xE0;
      cmd[2] = 0x00;
      cmd[3] = 0x01;
      cmd[4] = 0x51;
      serial.writeString(cmd, 5);
      res = serial.readChar();
      cout<< "range succ: " << std::hex << res <<endl;

      cmd[3] = 0xFF;
      do {
         cmd[0] = 0x55;
         cmd[1] = 0xE1;
         cmd[2] = 0x00;
         cmd[3] = 0x04;
         serial.writeString(cmd, 4);
         cmd[0] = serial.readChar();
         cmd[1] = serial.readChar();
         cmd[2] = serial.readChar();
         cmd[3] = serial.readChar();
      } while (cmd[3] == 0xFF);

      uint32_t range = (cmd[2]<<8)|cmd[3];
      cout<< "ret: " << std::dec <<  range <<endl;
   } catch(boost::system::system_error& e)
   {
      cout<<"Error: "<<e.what()<<endl;
      return 1;
   }
}
