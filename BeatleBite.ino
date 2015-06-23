#include <ProTrinketKeyboard.h> 

constexpr int sensorPin = A1;
constexpr int threshold = 80;
constexpr int samples = 250;
constexpr int sample_period =10000 / samples;

//init adc to bypass analogRead (overkill?)
void initChannel(){
    
    uint8_t pin = sensorPin;

    constexpr uint8_t analog_reference = DEFAULT;

    #if defined(analogPinToChannel)
        #if defined(__AVR_ATmega32U4__)
            if (pin >= 18) pin -= 18; // allow for channel or pin numbers
        #endif
        
        pin = analogPinToChannel(pin);
    
    #elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        if (pin >= 54) pin -= 54; // allow for channel or pin numbers
    #elif defined(__AVR_ATmega32U4__)
        if (pin >= 18) pin -= 18; // allow for channel or pin numbers
    #elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
        if (pin >= 24) pin -= 24; // allow for channel or pin numbers
    #else
        if (pin >= 14) pin -= 14; // allow for channel or pin numbers
    #endif

    #if defined(ADCSRB) && defined(MUX5)
        // the MUX5 bit of ADCSRB selects whether we're reading from channels
        // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
        ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
    #endif

    DIDR0 = 0x01; // turn off the digital input for adc0

    ADMUX = (analog_reference << 6) | (pin & 0x07);


            ADCSRA = 0;
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
    ADCSRA |= (1 << ADEN); //enable ADC
}

//get a read. oversample to increase resolution, and discard last bit to reduce noise
int getData(){
    constexpr int bit_over = 3;
    constexpr int oversample_len = pow(4, bit_over);

    int oversample[oversample_len]; 

    ADCSRA |= (1 << ADSC); // start the conversion
    while(ADCSRA & (1 << ADSC));    //wait and discard first lecture

    for(int i=0; i<oversample_len; i++){
        ADCSRA |= (1 << ADSC);
        while(ADCSRA & (1 << ADSC));

        byte m = ADCL; // fetch adc data
        byte j = ADCH;
        oversample[i] = (j << 8) | m; // form into an int
    }

    int acc=0;
    for(int i=0; i<oversample_len; i++){
        acc+=oversample[i];
    }

    return acc >> (bit_over+1);
}


void setup(){
    initChannel();
    delay(500);
    // start USB stuff
    TrinketKeyboard.begin();

    pinMode(13, OUTPUT);
}

uint16_t samples_buffer[samples];

void loop(){
    for(int i =0; i<samples; i++)
        samples_buffer[i]=0;

    long led_last_time=millis();
    int lstatus=LOW;
    //wait for first lecture above Threshold
    while(true){
        TrinketKeyboard.poll();

        //slow blink
        if(millis()-led_last_time >= 1000){
            led_last_time=millis();
            lstatus= (lstatus==LOW)? HIGH : LOW;
            digitalWrite(13, lstatus);
        }

        uint16_t value=getData();
        if(value>=threshold){
            samples_buffer[0]=value;
            break;
        }
    }

    int index=1;    
    long first_time=millis();

    //fill samples buffer
    long last_time=millis();
    while(index<samples){
        TrinketKeyboard.poll();
        
        //medium blink
        if(millis()-led_last_time >= 100){
            led_last_time=millis();
            lstatus= (lstatus==LOW)? HIGH : LOW;
            digitalWrite(13, lstatus);
        }

        //space evenly analog reads
        if(millis() - last_time < sample_period)
            continue;

        last_time=millis();

        int value= getData();

        samples_buffer[index]=value;
        ++index;
    }

    long running_time=millis()-first_time;

    //output the buffer as keypresses
    for(int i = 0; i<samples; ++i){
        //fast blink
        if(millis()-led_last_time >= 10){
            led_last_time=millis();
            lstatus= (lstatus==LOW)? HIGH : LOW;
            digitalWrite(13, lstatus);
        }
        TrinketKeyboard.println(samples_buffer[i]);
    }

    //write total acquisition time
    TrinketKeyboard.pressKey(0, KEYCODE_ARROW_UP);
    TrinketKeyboard.pressKey(0,0);
    TrinketKeyboard.pressKey(0, KEYCODE_ARROW_RIGHT);
    TrinketKeyboard.pressKey(0,0);
    TrinketKeyboard.print(running_time);
    TrinketKeyboard.pressKey(0, KEYCODE_ARROW_LEFT);
    TrinketKeyboard.pressKey(0,0);
    TrinketKeyboard.pressKey(0, KEYCODE_ARROW_DOWN);
    TrinketKeyboard.pressKey(0,0);

    delay(2000);
}