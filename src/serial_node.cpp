#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>

#define SERIAL_DEVICE "/dev/ttyACM1"
#define SERIAL_BAUDRATE 1000000

boost::asio::io_service io_service;
boost::asio::serial_port serial_port(io_service);
boost::asio::streambuf response_data;


void doReceive(void);
void doRecvCallBack(void);

void onRecvCallBack(boost::system::error_code ec, std::size_t bytes_transferred)
{
  std::cout << "bytes_transferred" <<bytes_transferred << std::endl;

  if (bytes_transferred > 0)
  {

    std::string buf_str;
    std::istream is(&response_data);
    is >> buf_str;

    if (buf_str.size() != 0)
    {

      buf_str.erase(std::remove(buf_str.begin(), buf_str.end(), '\n'), buf_str.end());
      std::cout << "buf_str" << buf_str << std::endl;

      bool cast_success = true;
      std::vector<std::string> str_vec;
      std::vector<int16_t> int_vec;
      boost::tokenizer<boost::char_separator<char>> tokens(buf_str, boost::char_separator<char>(",", "\n"));

      for (auto item : tokens)
      {
        try
        {
          int_vec.push_back(std::stoi(item));
        }
        catch (std::exception &e)
        {
          cast_success = false;
        }
      }

      if (cast_success == true && int_vec.size() == 8)
      {
        for (int i = 0; i < 8; i++)
        {
          std::cout << "param " << i << ":" << int_vec.at(i) << std::endl;
        }
      }
    }
  }
    doReceive();  
};

void doReceive(void)
{
  if (!serial_port.is_open())
  {
    return;
  }
  std::cout <<"doReceive"<<std::endl;
  boost::asio::async_read_until(serial_port, response_data, '\n', boost::bind(onRecvCallBack, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

int main()
{
  boost::asio::io_service io_service;

  serial_port.open(SERIAL_DEVICE);
  serial_port.set_option(boost::asio::serial_port_base::baud_rate(SERIAL_BAUDRATE));

  serial_port.write_some(boost::asio::buffer("3"));
  serial_port.write_some(boost::asio::buffer("s"));  

  doReceive();

  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

while(1);

  serial_port.write_some(boost::asio::buffer("0"));  
  return 0;
}