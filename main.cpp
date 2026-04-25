#include "stm32_simulator.hpp"
#include <cstdio>

void demo_gpio() {
    printf("\n=== GPIO Demo ===\n");
    auto& sim = initSim();
    auto* g = sim.gpio('A');
    if(!g){printf("No GPIOA\n");return;}
    g->cfg(5,GPIOMode::OUT_PP,GPIOSpeed::S50M);
    sim.start();
    for(int i=0;i<5;++i){
        sim.wreg(MM::GPIOA+16,1<<5);
        printf("ON %d\n",i);sim.runFor(500);
        sim.wreg(MM::GPIOA+20,1<<5);
        printf("OFF\n");sim.runFor(500);
    }
    sim.stop();sim.pstats();cleanupSim();
}

void demo_timer() {
    printf("\n=== Timer Demo ===\n");
    auto& sim = initSim();
    auto* t = sim.timer(2);
    if(!t){printf("No TIM2\n");return;}
    int c=0;
    TimerCfg cfg;cfg.psc=720;cfg.arr=999;
    cfg.cb=[&c](){if(++c%10==0)printf("Tick %d\n",c);};
    t->cfg(cfg);
    sim.start();sim.runFor(2000);sim.stop();
    printf("Ticks:%d\n",c);sim.pstats();cleanupSim();
}

void demo_usart() {
    printf("\n=== USART Demo ===\n");
    auto& sim = initSim();
    auto* u = sim.usart(1);
    if(!u){printf("No USART1\n");return;}
    USARTCfg cfg;cfg.baud=9600;
    cfg.rxCB=[](uint8_t d){printf("RX:%c\n",d);};
    u->cfg(cfg);
    sim.start();
    const char*m="Hello!";
    for(const char*p=m;*p;++p){sim.send(1,*p);sim.step(1000);}
    sim.runFor(100);sim.stop();sim.pstats();cleanupSim();
}

void demo_adc() {
    printf("\n=== ADC Demo ===\n");
    auto& sim = initSim();
    auto* a = sim.adc(1);
    if(!a){printf("No ADC1\n");return;}
    ADCCfg cfg;cfg.res=12;
    cfg.cb=[](uint16_t v){printf("ADC:%d (%.2fV)\n",v,v/4095.0f*3.3f);};
    a->cfg(cfg);
    sim.start();
    float vs[]={0,1.65f,3.3f};
    for(float v:vs){
        printf("In:%.2fV\n",v);
        sim.ain(1,0,v);
        sim.wreg(MM::ADC1+8,1|(1<<22));
        sim.runFor(10);
    }
    sim.stop();sim.pstats();cleanupSim();
}

int main(int ac,char**av){
    printf("=== STM32 Simulator ===\n");
    if(ac>1){std::string a=av[1];
        if(a=="gpio")demo_gpio();
        else if(a=="timer")demo_timer();
        else if(a=="usart")demo_usart();
        else if(a=="adc")demo_adc();
        else{demo_gpio();demo_timer();demo_usart();demo_adc();}
    }else{demo_gpio();demo_timer();demo_usart();demo_adc();}
    printf("\nDone!\n");return 0;
}
