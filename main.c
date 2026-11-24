/**
 * @file main.c
 * @brief Embedded Opgave 5 - ATmega328PB Xplained Mini
 * @details To UART'er, ringbuffer, INT0, Timer0/1, OLED og EEPROM.
 *          Alle kommentarer er paa dansk.
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <stdint.h>
#include "ssd1306.h"

/* Testappen viser 115200 baud; samme hastighed paa begge porte. */
#define UART_UBRR 16U            // U2X=1 og UBRR=16: ca. 117647 baud (2,1 % fejl)
#define RX_SIZE 128U             // Potens af 2, saa indeks kan maskeres
#define RX_MASK (RX_SIZE - 1U)
#define DEBOUNCE_MS 30UL         // Kontaktprel: 30 millisekunder
#define OLED_ADDR 0x3CU
#define USERNAME ":Vejmanden:"

/* ISR = Interrupt Service Routine.
 * UART1 modtager data fra testprogrammet via CP2102.
 * UART0 sender bytes tilbage til testprogrammets anden COM-port.
 */
static volatile uint8_t rxBuffer[RX_SIZE];
static volatile uint8_t rxHead = 0, rxTail = 0;
static volatile uint8_t rxOverflow = 0;
static volatile uint8_t lostByte = 0;

/* Timer0 = millisekunder, Timer1 = sekunder. */
static volatile uint32_t ticksMs = 0;
static volatile uint32_t secondsRunning = 0;

/* Eksternt interrupt INT0 paa PD2 (tryk/slip). */
static volatile uint8_t edgePending = 0;
static volatile uint32_t edgeAtMs = 0;
static uint8_t nextIsPress = 1;
static uint32_t downCount = 0;
static uint32_t upCount = 0;
static uint8_t buttonChanged = 1;

/* Seneste fem modtagne databytes (ikke handshake-tegn). */
static uint8_t recent[5] = {0, 0, 0, 0, 0};
static uint8_t recentSize = 0, recentNext = 0;
static uint8_t dataChanged = 1;

/* Ekstra funktion: lagre antal gennemfoerte minutter i EEPROM. */
static uint32_t EEMEM savedMinutesEeprom;
static uint32_t savedMinutes = 0;
static uint32_t lastSavedMinute = 0;

/* Kommunikationsprotokol. */
typedef enum {
    WAIT_COLON,    // Venter paa ':'
    WAIT_SEMICOLON,// Venter paa ';' efter brugernavn
    STREAMING,     // Modtager binære bytes 0..255
    WAIT_RESET     // Venter paa '_' ved protokolfejl/overrun
} ProtocolState;
static ProtocolState protocol = WAIT_COLON;
static uint8_t expectedByte = 0;
static uint8_t oledAvailable = 0;

static void uart_init(void) {
    /* UBRR = USART Baud Rate Register; U2X = Double Speed. */
    UCSR0A = (1U << U2X0);
    UBRR0H = 0;
    UBRR0L = UART_UBRR;
    UCSR0B = (1U << TXEN0); // UART0 skal kun sende
    UCSR0C = (1U << UCSZ01) | (1U << UCSZ00); // 8N1

    UCSR1A = (1U << U2X1);
    UBRR1H = 0;
    UBRR1L = UART_UBRR;
    UCSR1B = (1U << RXEN1) | (1U << RXCIE1); // RX + interrupt
    UCSR1C = (1U << UCSZ11) | (1U << UCSZ10); // 8N1
}

static void uart0_send(uint8_t c) {
    /* UDRE = USART Data Register Empty. */
    while (!(UCSR0A & (1U << UDRE0))) {}
    UDR0 = c;
}

static void uart0_text(const char *text) {
    while (*text) uart0_send((uint8_t)*text++);
}

/* Modtagelse: ISR producerer, main() konsumerer ringbufferen. */
ISR(USART1_RX_vect) {
    uint8_t c = UDR1;
    uint8_t next = (uint8_t)((rxHead + 1U) & RX_MASK);
    if (next == rxTail) {
        if (!rxOverflow) lostByte = c;
        rxOverflow = 1;
    } else if (!rxOverflow) {
        rxBuffer[rxHead] = c;
        rxHead = next;
    }
}

static uint8_t rx_pop(uint8_t *value) {
    if (rxHead == rxTail) return 0;
    *value = rxBuffer[rxTail];
    rxTail = (uint8_t)((rxTail + 1U) & RX_MASK);
    return 1;
}

/* Lille heltals-konvertering uden printf (sparer flash og SRAM). */
static void number_to_text(uint32_t value, char *dst) {
    char reverse[10];
    uint8_t n = 0;
    do {
        reverse[n++] = (char)('0' + (value % 10UL));
        value /= 10UL;
    } while (value != 0UL);
    while (n) *dst++ = reverse[--n];
    *dst = '\0';
}

static void timer_init(void) {
    /* WGM = Waveform Generation Mode; CTC = Clear Timer on Compare. */
    TCCR0A = (1U << WGM01);
    TCCR0B = (1U << CS01) | (1U << CS00); // prescaler 64
    OCR0A = 249;                         // 1 millisekund ved 16 MHz
    TIMSK0 = (1U << OCIE0A);             // Output Compare Interrupt Enable

    TCCR1A = 0;
    TCCR1B = (1U << WGM12) | (1U << CS12); // prescaler 256
    OCR1A = 62499;                          // 1 sekund ved 16 MHz
    TIMSK1 = (1U << OCIE1A);
}

ISR(TIMER0_COMPA_vect) { ticksMs++; }
ISR(TIMER1_COMPA_vect) {
    secondsRunning++;
    PINB = (1U << PINB1); // Toggle timer-LED paa PB1
}

static void button_init(void) {
    DDRD &= (uint8_t)~(1U << DDD2);   // PD2 input
    PORTD |= (1U << PORTD2);         // Intern pull-up
    DDRB |= (1U << DDB0) | (1U << DDB1); // LED paa PB0 og PB1
    PORTB &= (uint8_t)~((1U << PORTB0) | (1U << PORTB1));

    /* EICRA = External Interrupt Control Register A.
       ISC01:ISC00 = 10 giver faldende flanke paa INT0. */
    EICRA = (uint8_t)((EICRA | (1U << ISC01)) & ~(1U << ISC00));
    EIFR = (1U << INTF0);             // Ryd evt. ventende flag
    EIMSK |= (1U << INT0);           // External Interrupt Mask Register
}

ISR(INT0_vect) {
    EIMSK &= (uint8_t)~(1U << INT0); // Deaktiver midlertidigt under debounce
    edgeAtMs = ticksMs;
    edgePending = 1;
}

static void button_process(void) {
    uint8_t pending;
    uint32_t at, now;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pending = edgePending;
        at = edgeAtMs;
        now = ticksMs;
    }
    if (!pending || (uint32_t)(now - at) < DEBOUNCE_MS) return;

    uint8_t high = (PIND & (1U << PIND2)) != 0;
    if (nextIsPress && !high) {
        downCount++;
        nextIsPress = 0;
        PINB = (1U << PINB0); // Toggle knap-LED
        buttonChanged = 1;
    } else if (!nextIsPress && high) {
        upCount++;
        nextIsPress = 1;
        PINB = (1U << PINB0); // Toggle knap-LED
        buttonChanged = 1;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (nextIsPress) {
            EICRA = (uint8_t)((EICRA | (1U << ISC01)) & ~(1U << ISC00));
        } else {
            EICRA |= (1U << ISC01) | (1U << ISC00);
        }
        EIFR = (1U << INTF0); // Ryd kontaktprel
        edgePending = 0;
        EIMSK |= (1U << INT0);
    }
}

static void record_recent(uint8_t value) {
    recent[recentNext] = value;
    recentNext = (uint8_t)((recentNext + 1U) % 5U);
    if (recentSize < 5U) recentSize++;
    dataChanged = 1;
}

/* Ret navn paa tilstand, ikke data eller driverfiler. */
static void reset_protocol(void) {
    protocol = WAIT_COLON;
    expectedByte = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { rxTail = rxHead; }
    dataChanged = 1;
}

/* En overrun skal meldes med ;<mistet byte>; paa modsatte UART. */
static void handle_overrun(void) {
    uint8_t lost, occurred;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        occurred = rxOverflow;
        lost = lostByte;
        rxOverflow = 0;
        rxTail = rxHead;
    }
    if (!occurred) return;
    if (protocol == STREAMING) {
        char digits[11];
        number_to_text(lost, digits);
        uart0_send(';');
        uart0_text(digits);
        uart0_send(';');
        protocol = WAIT_RESET;
    } else {
        reset_protocol();
    }
}

static void process_byte(uint8_t c) {
    switch (protocol) {
        case WAIT_COLON:
            if (c == ':') {
                uart0_text(USERNAME);
                protocol = WAIT_SEMICOLON;
            }
            break;
        case WAIT_SEMICOLON:
            if (c == ';') {
                protocol = STREAMING;
                expectedByte = 0;
            } else if (c == ':') {
                uart0_text(USERNAME);
            } else if (c == '_') {
                reset_protocol();
            }
            break;
        case STREAMING:
            /* 95 = '_' er OGSAA en gyldig databyte i raekken 0..255.
               Kun en '_' uden for den forventede raekke er reset. */
            if (c == '_' && expectedByte != (uint8_t)'_') {
                reset_protocol();
                break;
            }
            record_recent(c);
            uart0_send(c); // Returner RAW byte 0..255 til testprogrammet
            expectedByte++;// uint8_t wrapper automatisk fra 255 til 0
            break;
        case WAIT_RESET:
            if (c == '_') reset_protocol();
            break;
    }
}

/* OLED: y er siden/linjen (0..7), x er pixelposition (0..127).
   Fontens bredde er 5 + en mellemrumskolonne. */
static void oled_line(uint8_t line, char *text) {
    if (!oledAvailable) return;
    if (SSD1306_SetPosition(0, line) != SSD1306_SUCCESS) return;
    SSD1306_DrawString("                     ", NORMAL);
    if (SSD1306_SetPosition(0, line) != SSD1306_SUCCESS) return;
    SSD1306_DrawString(text, NORMAL);
}

static void oled_number_line(uint8_t line, char *prefix, uint32_t value) {
    char lineText[22];
    char digits[11];
    uint8_t p = 0, n = 0;
    number_to_text(value, digits);
    while (prefix[p] && p < 20U) { lineText[p] = prefix[p]; p++; }
    while (digits[n] && p < 20U) lineText[p++] = digits[n++];
    lineText[p] = '\0';
    oled_line(line, lineText);
}

static void oled_recent(void) {
    char line[21];
    uint8_t p = 0;
    for (uint8_t i = 0; i < 5U; i++) {
        uint8_t v = 0;
        if (i < recentSize) {
            uint8_t first = (uint8_t)((recentNext + 5U - recentSize) % 5U);
            v = recent[(first + i) % 5U];
        }
        line[p++] = (char)('0' + v / 100U);
        line[p++] = (char)('0' + (v / 10U) % 10U);
        line[p++] = (char)('0' + v % 10U);
        if (i < 4U) line[p++] = ' ';
    }
    line[p] = '\0';
    oled_line(3, line);
}

static uint32_t elapsed_snapshot(void) {
    uint32_t snap;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { snap = secondsRunning; }
    return snap;
}

int main(void) {
    /* Laes gemte minutter; 0xFFFFFFFF er typisk tom EEPROM. */
    savedMinutes = eeprom_read_dword(&savedMinutesEeprom);
    if (savedMinutes == UINT32_MAX) savedMinutes = 0;

    uart_init();
    timer_init();
    button_init();
    oledAvailable = (SSD1306_Init(OLED_ADDR) == SSD1306_SUCCESS);
    if (oledAvailable) oledAvailable = (SSD1306_ClearScreen() == SSD1306_SUCCESS);
    sei(); // Global Interrupt Enable

    oled_line(4, "Embedded Styrer !!!");
    oled_line(5, "Bruger: Vejmanden");
    oled_number_line(0, "DOWN INT: ", downCount);
    oled_number_line(1, "UP INT: ", upCount);
    oled_number_line(2, "TIME: ", savedMinutes * 60UL);
    oled_recent();
    oled_line(6, "UART1 -> UART0");
    oled_line(7, "Afventer :");

    uint32_t shownSeconds = UINT32_MAX;
    uint8_t shownProtocol = 255;
    while (1) {
        handle_overrun();

        /* Behandl f.eks. hoejst 32 bytes ad gangen, saa andet koerer. */
        uint8_t value, batch = 32;
        while (batch-- && rx_pop(&value)) {
            process_byte(value);
            if (rxOverflow) break;
        }
        button_process();

        uint32_t elapsed = elapsed_snapshot();
        if (elapsed != shownSeconds) {
            shownSeconds = elapsed;
            uint32_t completedMinutes = elapsed / 60UL;
            if (completedMinutes > lastSavedMinute) {
                lastSavedMinute = completedMinutes;
                eeprom_update_dword(&savedMinutesEeprom,
                                    savedMinutes + completedMinutes);
            }
            oled_number_line(2, "TIME: ", savedMinutes * 60UL + elapsed);
            oled_recent(); // Seneste fem bytes opdateres hvert sekund
            dataChanged = 0;
        }

        if (buttonChanged) {
            buttonChanged = 0;
            oled_number_line(0, "DOWN INT: ", downCount);
            oled_number_line(1, "UP INT: ", upCount);
        }

        if (shownProtocol != (uint8_t)protocol) {
            shownProtocol = (uint8_t)protocol;
            switch (protocol) {
                case WAIT_COLON: oled_line(7, "Afventer :"); break;
                case WAIT_SEMICOLON: oled_line(7, "Afventer ;"); break;
                case STREAMING: oled_line(7, "Ringbuffer aktiv"); break;
                case WAIT_RESET: oled_line(7, "Overrun - vent _"); break;
            }
        }
    }
}
