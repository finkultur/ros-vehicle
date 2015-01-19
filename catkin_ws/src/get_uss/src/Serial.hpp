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

#ifndef SERIAL_H
#define SERIAL_H

#include <boost/asio.hpp>
#include <iostream>

class Serial {
  private:
    boost::asio::io_service* io;
    boost::asio::serial_port* serial_port;
    boost::asio::deadline_timer* timer;
    bool loop;
    char c;

  public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    Serial(std::string port, unsigned int baud_rate) {
      io = new boost::asio::io_service();
      serial_port = new boost::asio::serial_port(*io);
      serial_port->open(port);
      serial_port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    ~Serial() {
      serial_port->close();
      delete io;
      delete serial_port;
      delete timer;
    }

    void writeString(uint8_t* s, size_t lenght) {
        boost::asio::write(*serial_port,boost::asio::buffer(s,lenght));
    }

    // Called when an async read completes or has been cancelled
    void read_complete(const boost::system::error_code& error, 
                       size_t bytes_transferred) {        
      // Read has finished, so cancel the timer
      timer->cancel();
    }
    
    void time_out(const boost::system::error_code& error) {
      if(c == 0) {
        serial_port->cancel();
      }
    }

    char readChar() {
      timer = new boost::asio::deadline_timer(serial_port->get_io_service());

      c = 0;
      serial_port->get_io_service().reset();
      boost::asio::async_read(*serial_port, boost::asio::buffer(&c, 1), 
                              boost::bind(&Serial::read_complete, 
                              this, 
                              boost::asio::placeholders::error, 
                              boost::asio::placeholders::bytes_transferred)); 

      timer->expires_from_now(boost::posix_time::milliseconds(500));
      timer->async_wait(boost::bind(&Serial::time_out,
                        this, 
                        boost::asio::placeholders::error));

      serial_port->get_io_service().run();

      return c;
    }
};

#endif

