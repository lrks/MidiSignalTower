/******************************************************************************/
/*                                                                            */
/*                        MIDI Software for SignalTower                       */
/*                               -- Version 2 --                              */
/*                                                                            */
/******************************************************************************/
#include "mbed.h"
#include "ymz294.h"

//#define DEBUG
#define UNDEF_NOTE  128
#define UNDEF_CH    0xff
#define STR_BUF_SIZE    32
#define DEMO_INTERVAL   500

struct Slot {
    uint8_t ch;
    uint8_t note;
};

Slot GLOBAL_SLOT[3];
int GLOBAL_STATE = 0;
uint8_t GLOBAL_RUNNING = 0x00;
bool GLOBAL_IS_ACCEPT_DEMO = false;
bool GLOBAL_IS_RUNNING_DEMO = false;

DigitalOut red(dp27);
DigitalOut yellow(dp5);
DigitalOut green(dp18);
YMZ294 ymz(dp1, dp2, dp4, dp6, dp9, dp10, dp11, dp13, dp26, dp25, dp17);
RawSerial sp(dp16, dp15); // tx, rx
InterruptIn crash(dp28);
Ticker dog;
Timer fuwafuwatime;


/*------------------------------------*/
/*            Tower Control           */
/*------------------------------------*/
void reset(bool full=true);
void turnOn(int ch, uint8_t midi_ch, uint8_t note);
void turnOff(int ch);


/*------------------------------------*/
/*                MIDI                */
/*------------------------------------*/
void uartHandler();
void stateTransition(uint8_t recv, uint8_t *buf);
void midiIn(uint8_t *body);
int findCh(uint8_t midi_ch = UNDEF_CH, uint8_t note = UNDEF_NOTE);


/*------------------------------------*/
/*       Demo and KONOYONOOWARI       */
/*------------------------------------*/
void resetHandler();
void watch();
void demoHandler();

/*------------------------------------*/
/*              Use Xtal              */
/*------------------------------------*/
extern int stdio_retargeting_module;
extern "C" void $Sub$$SystemInit (void)
{
    // From mbed library
    //LPC_IOCON->PIO0_8 &= ~(0x3 << 3);
    //LPC_IOCON->PIO0_9 &= ~(0x3 << 3);
    //LPC_SWM->PINENABLE0 &= ~(0x3 << 4);
    LPC_SYSCON->PDRUNCFG &= ~(0x1 << 5);
    LPC_SYSCON->SYSOSCCTRL = 0x00;
    int i;
    for (i = 0; i < 200; i++) __NOP();

    // select the PLL input
    LPC_SYSCON->SYSPLLCLKSEL  = 0x1;                // Select PLL Input source 0=IRC, 1=OSC
    LPC_SYSCON->SYSPLLCLKUEN  = 0x01;               // Update Clock Source
    LPC_SYSCON->SYSPLLCLKUEN  = 0x00;               // Toggle Update Register
    LPC_SYSCON->SYSPLLCLKUEN  = 0x01;
    while (!(LPC_SYSCON->SYSPLLCLKUEN & 0x01));     // Wait Until Updated

    // Power up the system PLL
    LPC_SYSCON->SYSPLLCTRL    = 0x00000023;
    LPC_SYSCON->PDRUNCFG     &= ~(1 << 7);          // Power-up SYSPLL
    while (!(LPC_SYSCON->SYSPLLSTAT & 0x01));       // Wait Until PLL Locked

    // Select the main clock source
    LPC_SYSCON->MAINCLKSEL    = 0x3;                // Select main Clock source, 0=IRC, 1=PLLin, 2=WDO, 3=PLLout
    LPC_SYSCON->MAINCLKUEN    = 0x01;               // Update MCLK Clock Source
    LPC_SYSCON->MAINCLKUEN    = 0x00;               // Toggle Update Register
    LPC_SYSCON->MAINCLKUEN    = 0x01;
    while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait Until Updated

    LPC_SYSCON->SYSAHBCLKDIV  = 0x00000001;

    // System clock to the IOCON needs to be enabled or
    // most of the I/O related peripherals won't work.
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<16);
    //stdio_retargeting_module = 1;
}

/*------------------------------------*/
/*          Main Application          */
/*------------------------------------*/
int main() {
    #define LAMP(r, y, g)   red=(r);yellow=(y);green=(g)

    // UART
    sp.format(8, RawSerial::None, 1);
    sp.baud(31250);
    #ifdef DEBUG
        sp.puts("Welcome to Underground\r\n");
    #endif
    NVIC_SetPriority(UART_IRQn, 0);

    // YMZ294 Hardware
    reset();
    ymz.setVolume(CHANNEL_A, 0xf);
    ymz.setTone(CHANNEL_A, 2000.0);
    wait_ms(125);
    ymz.setTone(CHANNEL_A, 1000);
    wait_ms(100);
    ymz.muffleTone(CHANNEL_A);

    // Lamp
    for (int i=0; i<4; i++) {
        LAMP(i < 2, i == 0 || i == 2, i == 0 || i == 3);
        wait(1);
    }
    reset();

    // KONOYONOOWARI
    crash.mode(PullUp);
    crash.fall(&resetHandler);
    NVIC_SetPriority(EINT3_IRQn, 5);

    // Demo
    dog.attach(&watch, DEMO_INTERVAL);
    NVIC_SetPriority(TIMER_16_0_IRQn, 10);
    NVIC_SetPriority(TIMER_16_1_IRQn, 11);
    NVIC_SetPriority(TIMER_32_0_IRQn, 12);
    NVIC_SetPriority(TIMER_32_1_IRQn, 13);

    // MIDI-IN
    fuwafuwatime.start();
    sp.attach(&uartHandler, RawSerial::RxIrq);
}


/*----------------------------------------------------------------------------*/
/*                                Tower Control                               */
/*----------------------------------------------------------------------------*/
void reset(bool full) {
    #ifdef DEBUG
        sp.puts("[RESET] ");
        if (full)
            sp.puts("Full");
        else
            sp.puts("Light");
        sp.puts("\r\n");
    #endif

    GLOBAL_STATE = 0;
    if (full) GLOBAL_RUNNING = 0x00;
    for (int i=0; i<3; i++) turnOff(i);

    fuwafuwatime.reset();
    GLOBAL_IS_ACCEPT_DEMO = false;
    GLOBAL_IS_RUNNING_DEMO = false;

    ymz.reset();
    ymz.setMixer(NONE, NONE, NONE, TONE_C, TONE_B, TONE_A);
    //ymz.setEnvelope(255.0, 0x0);
}

void turnOn(int ch, uint8_t midi_ch, uint8_t note) {
    bool on = (midi_ch != UNDEF_CH && note != UNDEF_NOTE);

    if (ch == 0)
        red = on;
    else if (ch == 1)
        yellow = on;
    else if (ch == 2)
        green = on;

    GLOBAL_SLOT[ch].ch = on ? midi_ch : UNDEF_CH;
    GLOBAL_SLOT[ch].note = on ? note : UNDEF_NOTE;

    if (on) {
        ymz.setNote((Ch)ch, note);
        #ifdef DEBUG
            sp.puts("\tON\r\n");
        #endif
    } else {
        ymz.muffleTone((Ch)ch);
        #ifdef DEBUG
            sp.puts("\tOFF\r\n");
        #endif
    }
}

void turnOff(int ch) {
    turnOn(ch, UNDEF_CH, UNDEF_NOTE);
}


/*----------------------------------------------------------------------------*/
/*                                    MIDI                                    */
/*----------------------------------------------------------------------------*/
void uartHandler() {
    #define BUFSIZE 16
    uint8_t buf[BUFSIZE];
    static int idx = 0;

    #ifdef DEBUG
        sp.puts("----\r\n");
    #endif

    // for Demo
    if (GLOBAL_IS_RUNNING_DEMO) {
        reset(false);
    } else {
        fuwafuwatime.reset();
    }

    // Recieve
    uint8_t recv = sp.getc();
    #ifdef DEBUG
        char str[STR_BUF_SIZE];
        snprintf(str, STR_BUF_SIZE, "RECV: (0x%X, %d)\r\n", recv, GLOBAL_STATE);
        sp.puts(str);
    #endif

    // Real-time message
    if ((recv & 0xf8) == 0xf8) {
        #ifdef DEBUG
            sp.puts("Realtime Message\r\n--------\r\n");
        #endif
        return;
    }

    // Control
    if (GLOBAL_STATE == 0) {
        idx = 0;
        if (GLOBAL_RUNNING == 0x00 || (recv & 0x80) == 0x80) {
            GLOBAL_RUNNING = recv;
        } else {
            buf[idx++] = GLOBAL_RUNNING;
            stateTransition(GLOBAL_RUNNING, buf);
        }
    }

    buf[idx++] = recv;
    if (idx >= BUFSIZE) idx = BUFSIZE - 1;
    stateTransition(recv, buf);
}

void stateTransition(uint8_t recv, uint8_t *buf) {
    uint8_t ev;

    switch(GLOBAL_STATE) {
    case 0:
        ev = recv & 0xf0;
        GLOBAL_STATE =
            ((ev & 0xc0) == 0x80 || ev == 0xe0 || recv == 0xf2) ? 1 :
            (ev == 0xc0 || ev == 0xd0 || recv == 0xf1 || recv == 0xf3) ? 3 :
            (recv == 0xf0) ? 2 : 0;
        break;
    case 1:
        GLOBAL_STATE = 3;
        break;
    case 2:
        if (recv != 0xf7) break;
    case 3:
        midiIn(buf);
        GLOBAL_STATE = 0;
        break;
    }

    #ifdef DEBUG
        char str[STR_BUF_SIZE];
        snprintf(str, STR_BUF_SIZE, "State Transition: %d\r\n", GLOBAL_STATE);
        sp.puts(str);
    #endif
}


void midiIn(uint8_t *body) {
    uint8_t msg = body[0];
    uint8_t ev = msg & 0xf0;

    #ifdef DEBUG
        char str[STR_BUF_SIZE];
        snprintf(str, STR_BUF_SIZE, "\tMsg: 0x%X, Event: 0x%X\r\n", msg, ev);
        sp.puts(str);
    #endif

    // Control Change
    if (ev == 0xb0) {
        uint8_t cc = body[1];
        if (cc == 120 || (122 <= cc && cc <= 127)) {
            for (int i=0; i<3; i++) turnOff(i);
        }
        return;
    }

    // NoteOn or NoteOff
    if (!(ev == 0x80 || ev == 0x90 || ev == 0xa0)) return;

    uint8_t note = body[1];
    uint8_t velocity = body[2];
    if ((ev == 0x90 || ev == 0xa0) && velocity == 0x0) {
        msg &= 0x8f;
        ev = 0x80;

        #ifdef DEBUG
            snprintf(str, STR_BUF_SIZE, "\t--> [Change]Msg: 0x%X\r\n", msg);
            sp.puts(str);
        #endif
    }
    uint8_t midi_ch = msg & 0x0f;

    int ymz_ch = (ev == 0x90) ? findCh() : findCh(midi_ch, note);
    if (ymz_ch == -1) return;

    switch(ev) {
    case 0x80:  // NoteOff
        turnOff(ymz_ch);
        break;
    case 0x90:  // NoteOn
    case 0xa0:  // Polyphonic Key Pressure
        ymz.setVolume((Ch)ymz_ch, 0xf);
        if (ev == 0xa0) break;
        turnOn(ymz_ch, midi_ch, note);
        break;
    }
}

int findCh(uint8_t midi_ch, uint8_t note) {
    for (int i=0; i<3; i++) {
        if (GLOBAL_SLOT[i].ch == midi_ch && GLOBAL_SLOT[i].note == note) return i;
    }
    return -1;
}


/*----------------------------------------------------------------------------*/
/*                           Demo and KONOYONOOWARI                           */
/*----------------------------------------------------------------------------*/
void resetHandler() {
    reset();
}

void watch() {
    if (fuwafuwatime.read() < DEMO_INTERVAL) return;
    if (GLOBAL_IS_RUNNING_DEMO) return;

    GLOBAL_IS_ACCEPT_DEMO = true;
    GLOBAL_IS_RUNNING_DEMO = true;

    demoHandler();
    if (GLOBAL_IS_RUNNING_DEMO) reset(false);
}

void sound(int num);
void sort(int num);
void demoHandler() {
    if (!GLOBAL_IS_ACCEPT_DEMO) return;
    wait(5);

    #ifdef DEBUG
        char str[STR_BUF_SIZE];
        sp.puts("[demoHandler]\r\n");
    #endif

    srand(fuwafuwatime.read_us());
    int r = rand() % 4;
    #ifdef DEBUG
        snprintf(str, STR_BUF_SIZE, "r = %d\r\n", r);
        sp.puts(str);
    #endif

    switch(r) {
    case 0: // Knight
    case 1: // KOKYOU
        sound(r);
        break;
    case 2:
    case 3:
        sort(r - 2);
        break;
    }

    #ifdef DEBUG
        snprintf(str, STR_BUF_SIZE, "demoEnd, f1=%d\r\n", is_accept_demo);
        sp.puts(str);
    #endif

    demoHandler();
}

void sound(int num) {
    // Knight
    uint8_t knight_note[] = {
        #include "midi/knight_note.txt"
        0xff
    };

    float knight_duration[] = {
        #include "midi/knight_duration.txt"
        -1.0
    };

    // Kokyou
    uint8_t kokyou_note[] = {
        #include "midi/kokyou_note.txt"
        0xff
    };

    float kokyou_duration[] = {
        #include "midi/kokyou_duration.txt"
        -1.0
    };

    uint8_t *note_p;
    float *duration_p;

    switch (num) {
    case 0:
        note_p = knight_note;
        duration_p = knight_duration;
        break;
    case 1:
    default:
        note_p = kokyou_note;
        duration_p = kokyou_duration;
        break;
    }

    for (int i=0; (note_p[i >> 1] != 0xff && duration_p[i] != -1.0); i++) {
        if (!GLOBAL_IS_ACCEPT_DEMO) return;
        if (i % 2 == 0) {
            ymz.setVolume(CHANNEL_A, 0xf);
            turnOn(0, 0x0, note_p[i >> 1]);
            yellow = 1;
            green = 1;
        } else {
            turnOff(0);
            yellow = 0;
            green = 0;
        }
        wait(duration_p[i]);
    }

    // WASURETA
    turnOff(0);
    yellow = 0;
    green = 0;
}

bool isSoted(uint8_t *data, int n) {
    for (int i=0; i<(n-1); i++) {
        if (data[i] > data[i+1]) return false;
    }
    return true;
}

void swap(uint8_t *n1, uint8_t *n2) {
    uint8_t tmp = *n1;
    *n1 = *n2;
    *n2 = tmp;

    ymz.setVolume(CHANNEL_A, 0xf);
    turnOn(0, 0x0, *n1);
    yellow = 1;
    green = 1;

    wait_ms(150);

    turnOff(0);
    yellow = 0;
    green = 0;

    wait_ms(150);
}

void bogoSort(uint8_t *data, int n) {
    while (!isSoted(data, n)) {
        if (!GLOBAL_IS_ACCEPT_DEMO) return;
        int i1, i2 = rand() % n;
        do {
            i1 = rand() % n;
        } while (i1 == i2);
        swap(&data[i1], &data[i2]);
    }
}

void quickSort(uint8_t *data, int left, int right) {
    int pivot = data[(left + right) >> 1];
    int l = left, r = right;

    while (1) {
        if (!GLOBAL_IS_ACCEPT_DEMO) return;
        while ((data[l] < pivot) && (l < right)) l++;
        while ((data[r] > pivot) && (left < r)) r--;
        if (l > r) break;
        swap(&data[l++], &data[r--]);
    }

    if (left < r) quickSort(data, left, r);
    if (l < right) quickSort(data, l, right);
}

void sort(int num) {
    #define QUICK_DATA_SIZE 60
    #define BOGO_DATA_SIZE  5
    uint8_t data[QUICK_DATA_SIZE];  // QUICK_DATA_SIZE >= BOGO_DATA_SIZE

    switch(num) {
    case 0:
        for (int i=0; i<BOGO_DATA_SIZE; i++) data[i] = 50 + (rand() % 50);
        bogoSort(data, BOGO_DATA_SIZE);
        break;
    case 1:
    default:
        for (int i=0; i<BOGO_DATA_SIZE; i++) data[i] = 40 + (rand() % 60);
        quickSort(data, 0, QUICK_DATA_SIZE - 1);
        break;
    }
}

