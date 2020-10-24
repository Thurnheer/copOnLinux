#include<iostream>
#include <cop/TransportLinkLayer.hpp>
#include <cop/DataLinkLayer.hpp>
#include "uart.hpp"

enum Events {
    eHelloWorld
};

struct HelloWorld : cop::Event<eHelloWorld> {
    char H = 'H';
    char e = 'e';
    char l = 'l';
    char ll = 'l';
    char o = 'o';
    char s = ' ';
    char W = 'W';
    char oo = 'o';
    char r = 'r';
    char lll = 'l';
    char d = 'd';
    char n = '\n';

    template<class Coder>
    auto parse(Coder coder) {
        return coder | H | e | l | ll | o | s | W | o | r | lll | d | n;
    }
};

class Handler {
public:
    void handle(HelloWorld& hw) {
        std::cout << "Hw received:\n"
        << hw.H << hw.e << hw.l << hw.ll<< hw.o << hw.s << hw.W << hw.o << hw.r << hw.lll << hw.d << hw.n;
    }
};

class Channel {
public:
    Channel(Handler& handler) : buffer_(), uart_(), tll_(handler) {}

    void sendEvent(HelloWorld& hw) {
        auto it = buffer_.begin(); auto end = buffer_.end();
        tll_.sendEvent(hw, it, end);
        cop::DataLinkLayer<ReadIt> dll_(it, end);
        dll_.send(uart_);
    }

    void receive() {
        uart_.read();
        auto it = uart_.read_buf_.begin(); auto end = uart_.read_buf_.end();
        cop::DataLinkLayer dll(it, end);
        for(auto i : uart_.read_buf_) {
            if(cop::ProtocolErrc::success == dll.receive(i)) {
                break;
            }
        }
        tll_.receive(it, end);
    }
        
private:
    static constexpr size_t size_ = 128;
    std::vector<std::byte> buffer_;
    using ReadIt = decltype(buffer_)::iterator;
    using Itr = decltype(buffer_)::const_iterator;
    Uart<ReadIt> uart_;
    cop::TransportLinkLayer<Handler, ReadIt, std::tuple<HelloWorld> >tll_;

};

int main()
{
    //Uart<int> uart;
    Handler h;
    Channel c(h);

    for(int i = 0; i < 50; i++)
    {
        //uart.read();
        c.receive();
        sleep(1);
    }
}

