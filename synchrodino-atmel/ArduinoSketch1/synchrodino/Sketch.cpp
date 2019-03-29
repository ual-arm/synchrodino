/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/* Project Name: SYNCHRODINO
 * Authors: Antonio Gomez Fernandez
 * 
 * Copyright (C) 2019 - University of Almeria
 * Licence: GNU GPL v3
 * 
 * Compile for:
 *  Board: Arduino Mega 2560
 *  Microcontroller: Atmega 2560
 */

// librerias
#include <PriUint64.h>
#include <Arduino.h>
#include <stdlib.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//Beginning of Auto generated function prototypes by Atmel Studio
ISR(TIMER2_COMPA_vect );
void setup_timer2();
void pps();
void externa();
//End of Auto generated function prototypes by Atmel Studio



// Beginning of Auto generated function prototypes by Atmel Studio
// End of Auto generated function prototypes by Atmel Studio
// el primero es el Rxpin y el segundo es el Txpin

const uint64_t TICKS_PER_SECOND = 1000;

TinyGPSPlus gps;
// Message uC -> PC
struct Msg {
  uint64_t timestamp;
  uint8_t ID_sensor;
};

// Circular buffer: Codigo para crear una cola por si tubiera la necesidad de acumular mensajes
// https://github.com/jlblancoc/claraquino/blob/master/libclaraquino/circular_buffer.h
// GNU GPLv3
template <class T, uint8_t SIZE> class circular_buffer {
private:
  T m_data[SIZE];
  uint8_t m_next_read, m_next_write;

public:
  circular_buffer() : m_next_read(0), m_next_write(0) {}

  /** Insert a copy of the given element in the buffer.
    * \return false If the buffer run out of space.
    */
  bool push(const T &d) {
    uint8_t next_wr_idx = m_next_write + 1;
    if (next_wr_idx >= SIZE)
      next_wr_idx = 0;
    bool buf_overflow = (next_wr_idx == m_next_read);
    if (!buf_overflow) {
      m_data[m_next_write] = d;
      m_next_write = next_wr_idx;
    }
    return !buf_overflow;
  }

  /** Retrieve an element from the buffer.
    * \return false if the buffer was empty.
    */
  bool pop(T &out_val) {
    if (m_next_read == m_next_write)
      return false;

    out_val = m_data[m_next_read++];
    if (m_next_read >= SIZE)
      m_next_read = 0;
    return true;
  }

  /** Return the number of elements available for read ("pop") in the buffer
   * (this is NOT the maximum size of the internal buffer)
    * \sa capacity */
  uint8_t size() const {
    if (m_next_write >= m_next_read)
      return m_next_write - m_next_read;
    else
      return m_next_write + (SIZE - m_next_read);
  }

  /** Return the maximum capacity of the buffer.
    * \sa size
    */
  uint8_t capacity() const { return SIZE; }

  /** The maximum number of elements that can be written ("push") without rising
   * an overflow error.
    */
  uint8_t available() const { return (capacity() - size()) - 1; }

  /** Delete all the stored data, if any. */
  void clear() { m_next_write = m_next_read = 0; }

}; // end class circular_buffer

circular_buffer<Msg, 5> cola;
//Maxima cola de 5
// CONSTANTES
uint64_t ticks = 0;
uint64_t next_rmc = 0;
uint64_t current_rmc = 0;
//uint32_t mili;
time_t rawtime;
// funcion para pasar el la fecha y hora actual a UTC
uint64_t gpsdatetounixB();


// INTERRUCION TIMER 2
ISR(TIMER2_COMPA_vect) { ticks++; }
//Configuracion de la frecuencia de reloj
void setup_timer2() {
  // CTC: WGM22:0 = 010b (2)
  // WGM2:0
  // WGM1:1
  // WGM0:0
  TCCR2A = 0x02;

  // Prescaler: 8 nuevo
  // CS2{0:2} = 011b
  TCCR2B = 0x03; // 00000011b

  // OCR2A
  OCR2A = 49;

  // Habil. mascara de ints: TIMSK2
  // |=: bitwise or
  // (1<<n) = 2^n
  TIMSK2 |= (1 << OCIE2A);
}
//preparamos los pines de entrada y la velocidad de los puertos series en baudios por segundo
void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  // put your setup code here, to run once:
  setup_timer2();
  Serial.begin(9600);
  // Estoy usando el serail 1ss.begin(9600); //pines 19 y 18
  Serial1.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), pps, RISING);
  attachInterrupt(digitalPinToInterrupt(3), externa, RISING);
}
//programa principal
void loop() {
  // put your main code here, to run repeatedly:
  // tronco del programa
  if (Serial1.available() > 0) {
    gps.encode(Serial1.read());

    if (gps.date.isUpdated()) {
     next_rmc = gpsdatetounixB();
     //Serial.print("comprobacionseria2222222 next=");
     Serial.println(PriUint64<DEC>(next_rmc,10));
     //Serial.print("next=");
     //Serial.println(next_rmc);
    }
    if (cola.size() > 0) {
      Serial.print("hora=");
      Msg msg;
      cola.pop(msg);
      Serial.print("ticks=");
      Serial.println(PriUint64<DEC>(ticks,10));
      // madar puerto serie
      // envaira por serai msg
    }

    // leer la fecha y hora segun el gps en utc y convertirla a unix time stamd
    // el resultado guardar en next_rmc
  }
}

// si hay mensajes concolados, coger el mensaje y enviarlos


void pps() {
  if (next_rmc!=0){
    ticks = 0;
    current_rmc = next_rmc;
    
  }
}

void externa() {
  if (current_rmc != 0) {
    uint64_t hora = uint64_t(current_rmc) +  ticks;
    // encolar el mensaje al terminar
    Msg msg;
    msg.timestamp = hora;
    msg.ID_sensor = 0;
    cola.push(msg);
  }
}

uint64_t gpsdatetounixB(){

//dias de cada mes      0  1 2  3  4   5    6   7   8   9  10   11
const int month2days[]={0,31,59,90,120,151,181,212,243,273,304,334};
uint64_t mili;

//Codigo para pasar fechar y hora a UTC
{
  uint64_t x=gps.date.year();
  uint64_t vis=(x/4)-(1970/4);
  x=(gps.date.year()-1970)*365+vis;
  x=x*24*60*60;
  mili=x*TICKS_PER_SECOND;
}

const uint64_t dias = month2days[gps.date.month()-1] + gps.date.day()-1;
uint64_t horas = 24*dias + gps.time.hour();
mili+= horas*60*60*TICKS_PER_SECOND;
mili+= gps.time.minute()*60*TICKS_PER_SECOND;
mili+=gps.time.second()*TICKS_PER_SECOND;

return mili;
}
