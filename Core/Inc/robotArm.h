/*
 * robotArm.h
 *
 *  Created on: 11 mar 2022
 *      Author: Mateusz Kozaczek
 */



#define STEPPER_MOTOR_MAX_FREQ_HZ	1000		//	Maksymalna częstotliość silnika krokowego
#define STEPPER_MOTOR_MIN_FREQ_HZ	1			//	Minimalna częstotliwość silnika krokowego

#define STEPPER_MOTOR_MAX_SPEED		100			//	Maksymalna szybkość silnika w %
#define SPEED_DEFAULT				50			//	Domyślna prędkość silnika w %

#define STEP_PER_REVOLUTION			200			//	Rozdzielczość silnika krokowego - liczba kroków na obrót
#define MICRO_STEP					16			//	Liczba mikrokroków

#define SMALL_GEAR					9					//	Liczba zębów małego koła
#define BIG_GEAR					32					//	Liczba zębów dużego koła
#define GEAR_RATIO					BIG_GEAR/SMALL_GEAR	//	Przełożenie przekładni


#define HIGH_ENDSTOP				-26*M_PI/180		//	Kąt pozycji krańcowej górnego ramienia
#define LOW_ENDSTOP					-47.39*M_PI/180		//	Kąt pozycji krańcowej dolnego ramienia
#define ROT_ENDSTOP					0					//	Kąt pozycji krańcowej obrotu bazy

#define HIGH_HOME					M_PI/2				//	Kąt pozycji wyjściowej górnego ramienia
#define LOW_HOME					0					//	Kąt pozycji wyjściowej dolnego ramienia
#define ROT_HOME					0					//	Kąt pozycji wyjściowej obrotu bazy

#define X_HOME						0					//	Pozycja wyjściowa - koordynata X
#define Y_HOME						120					//	Pozycja wyjściowa - koordynata Y
#define Z_HOME						120					//	Pozycja wyjściowa - koordynata Z

#include "main.h"
#include "tim.h"

typedef enum
{
	idle = 0,			//	Silnik krokowy w stanie bezczynności
	working = 1			//	Silnik krokowy w trakcie pracy
}stepper_mode;

typedef enum
{
	CCW = 0,			//	Kierunek obrotu silnika w lewo
	CW = 1				//	Kierunek obrotu silnika w prawo
}stepper_direction;

typedef enum
{
	open = 0,			//	Chwytak otwarty
	close = 1			//	Chwytak zamknięty
}gripper_state;

struct htim_s
{
	TIM_HandleTypeDef *htim;	//	Struktura timera
	uint32_t channel;			//	Kanał timera
};

struct htim_gripper_s
{
	TIM_HandleTypeDef *htim;	//	Struktura timera
};


struct GPIO_s
{
	GPIO_TypeDef* PORT;			// Port GPIO
	uint16_t PIN;				// Pin GPIO
};

struct stepper_s
{
	struct htim_s timer;			//	Struktura timera silnika krokowego
	stepper_mode mode;				//	Stan silnika krokowego (pracuje lub bezczynny)
	volatile uint32_t step_counter;	//	Licznik kroków
	struct GPIO_s dirGPIO;			//	Struktura dla pinu sterującego DIR silnika
	uint32_t steps_to_count;		//	Liczba kroków do zliczenia
	stepper_direction direction;	//	Kierunek obrotu silnika
	uint8_t speed;					//	Szybkość silnika w %
};

struct gripper_s
{
	struct htim_gripper_s timer;	//	Struktura timera chwytaka
	stepper_mode mode;				//	Stan silnika chwytaka (pracuje lub bezczynny)
	volatile uint32_t step_counter;	//	Licznik kroków
	uint32_t steps_to_count;		//	Liczba kroków do zliczenia
	uint8_t tick;					//	Numer taktu
	gripper_state state;			//	Stan chwytaka (otwary lub zamknięty)
};

struct robotArm_s
{
	struct stepper_s high_Stepper;		//	Struktura silnika wyższego ramienia high
	struct stepper_s low_Stepper;		//	Struktura silnika niższego ramienia low
	struct stepper_s rot_Stepper;		//	Struktura silnika obrotu bazy
	struct gripper_s gripper_Stepper;	//	Struktura chwytaka


	float Xmm; 				//
	float Ymm;				//	Aktualna pozycja robota Xmm, Ymm, Zmm
	float Zmm;				//


	float highBigGear;		//
	float lowBigGear;  		//	Aktualny kąt większego koła
	float rotBigGear; 		//

	float highSmallGear;	//
	float lowSmallGear;		//	Aktualny kąt małego koła
	float rotSmallGear;		//


	float calcHigh;			//
	float calcLow;			//	Obliczony kąt
	float calcRot;			//
};

/*
 *	Funkcja inicjalizująca silnik krokowy:
 *	- Przypisuje strukturze silnika timer, kanał timera
 *	- Przypisuje pin i port DIR
 *	- Ustawia prędkość na domyślną
*/
void stepper_init(struct stepper_s *_stepper, TIM_HandleTypeDef *_htim, uint32_t _channel, GPIO_TypeDef *_dirPortGPIO, uint16_t _dirPinGPIO);

/*
 * Nadaje obrót silnikiem o kąt wyrażony w stopniach:
 * - Ustawia tryb pracy silnika na working (w trakcie pracy)
 * - Ustawia prędkość silnika
 * - Oblicza i ustawia liczbę kroków do obrotu silnikiem na podstawie kąta w stopniach, dla kątów ujemnych zmienia stan wyjścia DIR
 * - Zeruje licznik kroków
 * - Załącza PWM w trybie przerwaniowym
*/
void stepper_set_angle_degree(struct stepper_s *_stepper, float _angle);

/*
 * Nadaje obrót silnikiem o kąt wyrażony w radianach:
 * - Ustawia tryb pracy silnika na working (w trakcie pracy)
 * - Ustawia prędkość silnika
 * - Oblicza i ustawia liczbę kroków do obrotu silnikiem na podstawie kąta w radianach, dla kątów ujemnych zmienia stan portu DIR
 * - Zeruje licznik kroków
 * - Załącza PWM w trybie przerwaniowym
*/
void stepper_set_angle_radian(struct stepper_s *_stepper, float _angle);

/*
 * Funkcja zatrzymująca silnik:
 * - Ustawia silnik w tryb idle (bezczynny)
 * - Wyłącza PWM tego silnika
*/
void stepper_stop(struct stepper_s *);
/*
 * Ustawia kierunek obrotu silnika - stan na wyjściu DIR
*/
void stepper_set_direction(struct stepper_s *, stepper_direction);

/*
 * Ustawia prędkość silnika w % częstotliwości maksymalnej
*/
void stepper_set_speed(struct stepper_s *, uint32_t);

/*
 *	Funkcja inicjalizująca robota:
 *	Załącza silniki - zeruje pin ENABLE
 *	Inicjalizuje 4 silniki - high, low, rot i gripper
 *	Przypisuje silnikom pozycje krańcowe - ENDSTOP
 *
*/
void robotArm_init(struct robotArm_s *);

/*
 *	Oblicza kąty obrotu silników high, low i rot na podstawie geometrii w zadanych pozycjach Xmm, Ymm, Zmm
*/
void robotArm_Geometry_calculateRadian(struct robotArm_s *_robotArm, float _Xmm, float _Ymm, float _Zmm);

/*
 *	Funkcja ruchu robota do zadanej pozycji Xmm,Ymm,Zmm:
 *	- Obliczenie pozycji silników
 *	- Obliczenie i wpisanie kątów obrotu silników
 *	- Oczekiwanie, aż wszystkie silniki zakończą pracę
 *	- Zapisanie aktualnej pozycji robota
*/
void robotArm_moveToXYZ(struct robotArm_s * _robotArm, float _Xmm, float _Ymm, float _Zmm);
/*
 *	Funkcja sprawdzająca, czy wszystkie silniki są w trybie idle (bezczynności)
 *	- Jeżeli high, low i rot są w trybie idle, zwraca 1, w przeciwnym wypadku zwraca 0
*/
unsigned int robotArm_isIdle(struct robotArm_s * );
/*
 *	Funkcja realizująca otwieranie/zamykanie chwytaka:
 *	- Jeżeli aktualny stan chwytaka jest taki sam jak zadany to omija działanie funkcji
 *	- Ustawia tryb chwytaka na working (w trakcie pracy)
 *	- Dla otwierania zadaje 500 kroków do wykonania, a dla zamykania 900
 *	- Zeruje licznik kroków
 *	- Załącza timer chwytaka w trybie przerwaniowym
 *	- Czeka, aż chwytak wykona zadanie
*/
void robotArm_Gripper(struct robotArm_s * , gripper_state );

/*
 * Funkcja zatrzymująca silnik chwytaka:
 * - Ustawia chwytak w tryb idle (bezczynny)
 * - Wyłącza timer chwytaka
*/
void gripper_stop(struct gripper_s * );
