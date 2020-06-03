#include "mbed.h"
#include "mbed_rpc.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "stdlib.h"
#include "math.h"
#include "mbed_events.h"

#define UINT14_MAX        16383
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7

RawSerial pc(USBTX, USBRX);
RawSerial xbee(D12, D11);
DigitalOut led1(LED1);
InterruptIn sw2(SW2);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread thread;
I2C i2c( PTD9,PTD8);
Serial pc(USBTX, USBRX);
int m_addr = FXOS8700CQ_SLAVE_ADDR1;
float x, y, z;

void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);
void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);
void FXOS8700CQ_writeRegs(uint8_t * data, int len);
void acc(Arguments *in, Reply *out);
RPCFunction rpcAcc(&acc, "acc");

void FXOS8700CQ_0(){
    pc.baud(115200);
    uint8_t who_am_i, data[2], res[6];
    int16_t A_acc16;
    float t[3];
    
    FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
    data[1] |= 0x01;
    data[0] = FXOS8700Q_CTRL_REG1;
    FXOS8700CQ_writeRegs(data, 2);
    FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);

    FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

    A_acc16 = (res[0] << 6) | (res[1] >> 2);
    if (A_acc16 > UINT14_MAX/2)
        A_acc16 -= UINT14_MAX;
    t[0] = ((float)A_acc16) / 4096.0f;

    A_acc16 = (res[2] << 6) | (res[3] >> 2);
    if (A_acc16 > UINT14_MAX/2)
        A_acc16 -= UINT14_MAX;
    t[1] = ((float)A_acc16) / 4096.0f;

    A_acc16 = (res[4] << 6) | (res[5] >> 2);
    if (A_acc16 > UINT14_MAX/2)
        A_acc16 -= UINT14_MAX;
    t[2] = ((float)A_acc16) / 4096.0f;

    x = t[0];
    y = t[1];
    z = t[2];
}

void acc(Arguments *in, Reply *out){
    pc.baud(115200);
    uint8_t who_am_i, data[2], res[6];
    int16_t A_acc16;
    float t[3], q;
    float velocity = 0;

    FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

    A_acc16 = (res[0] << 6) | (res[1] >> 2);
    if (A_acc16 > UINT14_MAX/2)
        A_acc16 -= UINT14_MAX;
    t[0] = ((float)A_acc16) / 4096.0f;

    A_acc16 = (res[2] << 6) | (res[3] >> 2);
    if (A_acc16 > UINT14_MAX/2)
        A_acc16 -= UINT14_MAX;
    t[1] = ((float)A_acc16) / 4096.0f;

    A_acc16 = (res[4] << 6) | (res[5] >> 2);
    if (A_acc16 > UINT14_MAX/2)
        A_acc16 -= UINT14_MAX;
    t[2] = ((float)A_acc16) / 4096.0f;

    q = pow( (pow(t[0], 2) + pow(t[1], 2)), 0.5);
    velocity = q * 0.1;

    led1 = !led1;
        
    pc.printf("velocity : %.4f\n", velocity);

    wait(0.1);

}

void btn_fall_irq(){
    queue.call(&acc);
}

int main() {
    char xbee_reply[4];

    pc.baud(115200);
    FXOS8700CQ_0();
      
    thread.start(callback(&queue, &EventQueue::dispatch_forever));
    sw2.rise(queue.event(btn_fall_irq));

    xbee.baud(9600);
    xbee.printf("+++");
    xbee_reply[0] = xbee.getc();
    xbee_reply[1] = xbee.getc();
    if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
        pc.printf("enter AT mode.\r\n");
        xbee_reply[0] = '\0';
        xbee_reply[1] = '\0';
    }

    // SLAVE_MY should be different between Slave A and Slave B
    xbee.printf("ATMY 0x240\r\n");
    reply_messange(xbee_reply, "setting MY : 0x240");

    xbee.printf("ATDL 0x140\r\n");
    reply_messange(xbee_reply, "setting DL : 0x140");

    xbee.printf("ATID 0x1\r\n");
    reply_messange(xbee_reply, "setting PAN ID : 0x1");

    xbee.printf("ATWR\r\n");
    reply_messange(xbee_reply, "write config");

    xbee.printf("ATMY\r\n");
    check_addr(xbee_reply, "MY");

    xbee.printf("ATDL\r\n");
    check_addr(xbee_reply, "DL");

    xbee.printf("ATCN\r\n");
    reply_messange(xbee_reply, "exit AT mode");
    xbee.getc();

    // start
    pc.printf("start\r\n");
    led1 = 1;
    thread.start(callback(&queue, &EventQueue::dispatch_forever));

    // Setup a serial interrupt function of receiving data from xbee
    xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
}

void xbee_rx_interrupt(void)
{
  xbee.attach(NULL, Serial::RxIrq); // detach interrupt
  queue.call(&xbee_rx);
}

void xbee_rx(void)
{
  char buf[100] = {0};
  char outbuf[100] = {0};
  while(xbee.readable()){
    for (int i=0; ; i++) {
      char recv = xbee.getc();
      if (recv == '\r') {
        break;
      }
      buf[i] = pc.putc(recv);
    }
    RPC::call(buf, outbuf);
    pc.printf("%s\r\n", outbuf);
    
    wait(0.1);
  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}

void reply_messange(char *xbee_reply, char *messange){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
    pc.printf("%s\r\n", messange);
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
  }
}

void check_addr(char *xbee_reply, char *messenger){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  xbee_reply[3] = xbee.getc();
  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
  xbee_reply[0] = '\0';
  xbee_reply[1] = '\0';
  xbee_reply[2] = '\0';
  xbee_reply[3] = '\0';
}

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}

void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}

