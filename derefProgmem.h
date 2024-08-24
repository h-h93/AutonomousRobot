//
//  derefProgmem.hpp
//  
//
//  Created by Hanif Hussain on 16/03/2016.
//
//

#include <Arduino.h>

template<typename T>
void PROGMEM_derefProgmem (const T* orig, T& destination){
    memcpy_p (&destination, orig, sizeof(T));
}

template<typename T>
T PROGMEM_getAnything(const T* orig){
    static T temp;
    memcpy_p (&temp, orig, sizeof(orig));
    return temp;
}