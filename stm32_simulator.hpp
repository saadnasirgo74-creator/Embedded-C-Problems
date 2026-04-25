#ifndef STM32_SIM_HPP
#define STM32_SIM_HPP
#include <cstdint>
#include <vector>
#include <array>
#include <functional>
#include <memory>
#include <queue>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <stdexcept>

constexpr uint64_t SYSCLK = 72000000;

namespace MM {
    constexpr uint32_t GPIOA=0x40010800,GPIOB=0x40010C00,GPIOC=0x40011000;
    constexpr uint32_t TIM2=0x40000000,TIM3=0x40000400;
    constexpr uint32_t USART1=0x40013800,USART2=0x40004400;
    constexpr uint32_t ADC1=0x40012400;
}

enum class GPIOMode{IN,OUT,OUT_PP,OUT_OD};
enum class GPIOSpeed{S2M,S10M,S50M};
enum class PinState{LOW,HIGH};
enum class SimStatus{STOP,RUN,PAUSE};

using TimerCB = std::function<void()>;
using USARTCB = std::function<void(uint8_t)>;
using ADCCB = std::function<void(uint16_t)>;

struct TimerCfg{uint32_t psc=1,arr=65535;TimerCB cb;};
struct USARTCfg{uint32_t baud=9600;USARTCB rxCB;};
struct ADCCfg{uint8_t res=12;ADCCB cb;};
struct Stats{uint64_t cycles=0,instr=0,gpio=0;double simT=0;};

class GPIO{
public:
    GPIO(char n):name(n){crl=crh=0x44444444;}
    uint32_t rd(uint32_t o){return o==0?crl:o==4?crh:o==8?idr:o==12?odr:0;}
    void wr(uint32_t o,uint32_t v){
        if(o==0)crl=v;else if(o==4)crh=v;
        else if(o==12){odr=v&0xFFFF;upd();}
        else if(o==16){odr|=(v&0xFFFF);odr&=~((v>>16)&0xFFFF);upd();}
        else if(o==20){odr&=~(v&0xFFFF);upd();}
    }
    void setPin(uint8_t p,PinState s){if(p<16)pins[p]=s;updi();}
    PinState getPin(uint8_t p)const{return p<16?pins[p]:PinState::LOW;}
    void cfg(uint8_t,GPIOMode,GPIOSpeed){}
private:
    char name;uint32_t crl,crh,idr=0,odr=0;
    std::array<PinState,16>pins{};
    void upd(){for(int i=0;i<16;++i)pins[i]=(odr>>i)&1?PinState::HIGH:PinState::LOW;}
    void updi(){idr=0;for(int i=0;i<16;++i)if(pins[i]==PinState::HIGH)idr|=(1<<i);}
};

class Timer{
public:
    Timer(uint8_t i,uint64_t c):id(i),clk(c){}
    uint32_t rd(uint32_t o){return o==0?cr1:o==36?cnt:o==44?arr:0;}
    void wr(uint32_t o,uint32_t v){if(o==0){cr1=v;run=v&1;}else if(o==40)psc=v;else if(o==44)arr=v&0xFFFF;}
    void cfg(const TimerCfg&cfg){psc=cfg.psc-1;arr=cfg.arr;cb=cfg.cb;}
    void upd(uint64_t now){if(!run)return;uint64_t dt=now-last;last=now;uint64_t inc=(clk*dt)/((psc+1)*1000000);if(cnt+inc>=arr){cnt=0;if(cb)cb();}else cnt+=inc;}
    uint8_t getId()const{return id;}
private:
    uint8_t id;uint64_t clk;uint32_t cr1=0,psc=0,arr=65535,cnt=0,last=0;bool run=false;TimerCB cb;
};

class USART{
public:
    USART(uint8_t i,uint64_t c):id(i),clk(c){sr=0xC0;}
    uint32_t rd(uint32_t o){if(o==0)return sr;if(o==4&&!rx.empty()){uint8_t d=rx.front();rx.pop();return d;}return 0;}
    void wr(uint32_t o,uint32_t v){if(o==4){tx.push(v&0xFF);sr|=0xC0;}if(o==12){cr1=v;if(v&0x2000)en=true;}}
    void cfg(const USARTCfg&cfg){brr=clk/cfg.baud;rxCB=cfg.rxCB;cr1=0x200C;en=true;}
    void recv(uint8_t d){rx.push(d);sr|=0x20;if(rxCB)rxCB(d);}
    bool get(uint8_t&d){if(rx.empty())return false;d=rx.front();rx.pop();return true;}
    uint8_t getId()const{return id;}
private:
    uint8_t id;uint64_t clk;uint32_t sr,cr1=0,brr=0;bool en=false;std::queue<uint8_t>tx,rx;USARTCB rxCB;
};

class ADC{
public:
    ADC(uint8_t i):id(i){}
    uint32_t rd(uint32_t o){return o==0?sr:o==76?dr:0;}
    void wr(uint32_t o,uint32_t v){if(o==8){cr2=v;if(v&1){en=true;if(v&(1<<22))conv();}}if(o==52)sqr3=v;}
    void cfg(const ADCCfg&cfg){res=cfg.res;cb=cfg.cb;}
    void set(uint8_t ch,float v){if(ch<18)analog[ch]=v;}
    uint8_t getId()const{return id;}
private:
    uint8_t id;uint32_t sr=0,cr2=0,sqr3=0,dr=0;uint8_t res=12;bool en=false;std::array<float,18>analog{};ADCCB cb;
    void conv(){uint8_t ch=sqr3&0x1F;float v=ch<18?analog[ch]:0;dr=((v/3.3f)*((1<<res)-1));sr=3;if(cb)cb(dr);}
};

class STM32Sim{
public:
    STM32Sim(uint64_t c=SYSCLK):clk(c){
        for(char c='A';c<='C';++c)gpios.push_back(std::make_unique<GPIO>(c));
        timers.push_back(std::make_unique<Timer>(2,clk));
        timers.push_back(std::make_unique<Timer>(3,clk));
        usarts.push_back(std::make_unique<USART>(1,clk));
        usarts.push_back(std::make_unique<USART>(2,clk));
        adcs.push_back(std::make_unique<ADC>(1));
    }
    void start(){st=SimStatus::RUN;t0=getus();t=0;}
    void stop(){st=SimStatus::STOP;ustats();}
    void pause(){if(st==SimStatus::RUN)st=SimStatus::PAUSE;}
    void resume(){if(st==SimStatus::PAUSE){st=SimStatus::RUN;t0=getus();}}
    void runFor(uint32_t ms){start();uint64_t e=t+ms*1000;while(t<e&&st==SimStatus::RUN)step(100);stop();}
    void step(uint64_t us){if(st!=SimStatus::RUN)return;t+=us;for(auto&x:timers)x->upd(t);s.cycles+=(clk*us)/1000000;s.instr+=s.cycles/3;}
    GPIO* gpio(char p){if(p<'A'||p>'C')return nullptr;return gpios[p-'A'].get();}
    Timer* timer(uint8_t i){for(auto&x:timers)if(x->getId()==i)return x.get();return nullptr;}
    USART* usart(uint8_t i){for(auto&x:usarts)if(x->getId()==i)return x.get();return nullptr;}
    ADC* adc(uint8_t i){for(auto&x:adcs)if(x->getId()==i)return x.get();return nullptr;}
    uint32_t rreg(uint32_t a){
        if(a>=MM::GPIOA&&a<=MM::GPIOC+0x24){char p='A'+(a-MM::GPIOA)/0x400;auto*g=gpio(p);return g?g->rd(a&0x3FF):0;}
        if(a>=MM::TIM2&&a<=MM::TIM3+0x44){uint8_t i=a>=MM::TIM3?3:2;auto*x=timer(i);return x?x->rd(a&0x3FF):0;}
        if(a>=MM::USART1&&a<=MM::USART2+0x24){uint8_t i=a>=MM::USART2?2:1;auto*x=usart(i);return x?x->rd(a&0x3FF):0;}
        if(a>=MM::ADC1&&a<=MM::ADC1+0x50){auto*x=adc(1);return x?x->rd(a&0x3FF):0;}
        return 0;
    }
    void wreg(uint32_t a,uint32_t v){
        if(a>=MM::GPIOA&&a<=MM::GPIOC+0x24){char p='A'+(a-MM::GPIOA)/0x400;auto*g=gpio(p);if(g)g->wr(a&0x3FF,v);}
        if(a>=MM::TIM2&&a<=MM::TIM3+0x44){uint8_t i=a>=MM::TIM3?3:2;auto*x=timer(i);if(x)x->wr(a&0x3FF,v);}
        if(a>=MM::USART1&&a<=MM::USART2+0x24){uint8_t i=a>=MM::USART2?2:1;auto*x=usart(i);if(x)x->wr(a&0x3FF,v);}
        if(a>=MM::ADC1&&a<=MM::ADC1+0x50){auto*x=adc(1);if(x)x->wr(a&0x3FF,v);}
        s.gpio++;
    }
    void pin(char p,uint8_t n,PinState st){auto*g=gpio(p);if(g)g->setPin(n,st);}
    PinState gpin(char p,uint8_t n){auto*g=gpio(p);return g?g->getPin(n):PinState::LOW;}
    void send(uint8_t i,uint8_t d){auto*x=usart(i);if(x)x->recv(d);}
    bool recv(uint8_t i,uint8_t&d){auto*x=usart(i);return x?x->get(d):false;}
    void ain(uint8_t i,uint8_t c,float v){auto*x=adc(i);if(x)x->set(c,v);}
    Stats stats()const{return s;}
    uint64_t time()const{return t;}
    SimStatus status()const{return st;}
    void pstats()const{printf("\n=== Stats ===\nStatus:%s\nCycles:%lu\nInstr:%lu\nGPIO:%lu\nSimT:%.3fs\n=============\n",st==SimStatus::RUN?"Run":st==SimStatus::PAUSE?"Pause":"Stop",s.cycles,s.instr,s.gpio,s.simT);}
private:
    uint64_t clk;SimStatus st=SimStatus::STOP;uint64_t t0,t;Stats s;
    std::vector<std::unique_ptr<GPIO>>gpios;
    std::vector<std::unique_ptr<Timer>>timers;
    std::vector<std::unique_ptr<USART>>usarts;
    std::vector<std::unique_ptr<ADC>>adcs;
    uint64_t getus(){return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();}
    void ustats(){s.simT=t/1e6;}
};

static std::unique_ptr<STM32Sim> g_sim;
STM32Sim& initSim(uint64_t c=SYSCLK){if(!g_sim)g_sim=std::make_unique<STM32Sim>(c);return *g_sim;}
void cleanupSim(){g_sim.reset();}
STM32Sim& getSim(){if(!g_sim)throw std::runtime_error("Call initSim first");return *g_sim;}

#endif
