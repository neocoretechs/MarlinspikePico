/*
 * pins.cpp
 * Keep track of pins assigned by the user during runtime
 * Created: 8/7/2014 11:01:12 AM
 *  Author: jg
 */
#include "pins.h"
#include <string.h>

uint assignedPins[100]; // 100 pins headroom
bool initialized = false;
const int sensitive_pins[] = SENSITIVE_PINS;
extern void *memset(void *, int, size_t);
/*
* Return true if pin was newly assigned, false if previously assigned
*/
bool assignPin(uint tpin) {
		if( !initialized) {
			initialized = true;
			memset(&assignedPins, 0, 100);
			for(int i = 0; i < (int)sizeof(sensitive_pins); i++) {
				if (sensitive_pins[i] != -1) {
					assignedPins[sensitive_pins[i]] = PIN_RESERVED;
				}
			 }
		}
		if( tpin > 99) return false;
		if( assignedPins[tpin] != 0 )
				return false;
		assignedPins[tpin] = PIN_ASSIGNED;
		return true;
}
/*
* Unassign pin regardless of its status
*/
bool unassignPin(uint tpin) {
	if( tpin > 99) return false;
	assignedPins[tpin] = 0;
	return true;
}
/*
* Return the status of the pin assignment: 0 - unassigned, PIN_ASSIGNED, PIN_RESERVED
*/
int pinAssignment(uint tpin) {
	if( tpin > 99 ) return -1;
	return assignedPins[tpin];
}