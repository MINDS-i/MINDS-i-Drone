/* Copyright 2021 MINDS-i, INC.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef BUMPER_H
#define BUMPER_H
#include "MINDSi.h"

#define DEFAULT_DEBOUNCE_DELAY 50
#define BUMPER_BUTTON_LEFT 0
#define BUMPER_BUTTON_RIGHT 1

class bumper
{
private:

	int8_t m_pin[2];
	int8_t m_buttonState[2];
	int8_t m_buttonEvent[2];
	int8_t m_lastButtonState[2];
	uint32_t m_lastDebounceTime[2];
	uint32_t m_debounceDelayAmount;
	
public:

	bumper()
	{
		m_pin[BUMPER_BUTTON_LEFT]=A5;
		m_pin[BUMPER_BUTTON_RIGHT]=A6;
		m_buttonState[BUMPER_BUTTON_LEFT] = 0;
		m_buttonState[BUMPER_BUTTON_RIGHT] = 0;
		m_buttonEvent[BUMPER_BUTTON_LEFT] = 0;
		m_buttonEvent[BUMPER_BUTTON_LEFT] = 0;
		m_lastButtonState[BUMPER_BUTTON_LEFT] = 0;
		m_lastButtonState[BUMPER_BUTTON_RIGHT] = 0;
		m_lastDebounceTime[BUMPER_BUTTON_LEFT] = 0;
		m_lastDebounceTime[BUMPER_BUTTON_RIGHT] = 0;

		m_debounceDelayAmount = DEFAULT_DEBOUNCE_DELAY;

		pinMode(m_pin[BUMPER_BUTTON_LEFT], INPUT);
		pinMode(m_pin[BUMPER_BUTTON_RIGHT], INPUT);

	}
	
	void begin(uint8_t leftPin, uint8_t rightPin)
	{		
		pinMode(leftPin, INPUT);
		pinMode(rightPin, INPUT);

		m_pin[BUMPER_BUTTON_LEFT] = leftPin;
		m_pin[BUMPER_BUTTON_RIGHT] = rightPin;
	}

	void updateButton(uint8_t pin_id)
	{
		uint8_t curButtonState;

		if (pin_id > 2)
			return;

		//invert state: pressed is logic level 0
		curButtonState = !digitalRead(m_pin[pin_id]);
		//Serial.println(curButtonState);

		if (curButtonState != m_lastButtonState[pin_id])
		{
			m_lastDebounceTime[pin_id] = millis();
		}
		else
		{
			if (millis() - m_lastDebounceTime[pin_id] > m_debounceDelayAmount)
			{
				if (curButtonState != m_buttonState[pin_id])
				{
					m_buttonState[pin_id] = curButtonState;
					m_buttonEvent[pin_id] = 1;
				}
			}
		}

		//save state
		m_lastButtonState[pin_id] = curButtonState;

	}

	void update()
	{
		updateButton(BUMPER_BUTTON_LEFT);
		updateButton(BUMPER_BUTTON_RIGHT);
	}

	uint8_t buttonEvent(uint8_t pin_id)
	{
		uint8_t event = m_buttonEvent[pin_id]; 
		//clear event
		m_buttonEvent[pin_id] = 0;
		return event;
	}

	uint8_t buttonState(uint8_t pin_id)
	{
		return m_buttonState[pin_id];
	}

	uint8_t leftButtonEvent() { return buttonEvent(BUMPER_BUTTON_LEFT); }
	uint8_t rightButtonEvent() { return buttonEvent(BUMPER_BUTTON_RIGHT); }
	
	uint8_t leftButtonState() { return buttonState(BUMPER_BUTTON_LEFT); }
	uint8_t rightButtonState() { return buttonState(BUMPER_BUTTON_RIGHT);}



};

#endif
