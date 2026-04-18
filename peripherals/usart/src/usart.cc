/*
 * usart.cc — Module registration for the USART peripheral.
 *
 * The module_register() symbol is the dynamic-library entry point called by
 * qbox's ModuleFactory when it loads usart.so.  It registers the Usart class
 * under the name "Usart", which must match the moduletype field in the
 * platform's Lua configuration file.
 */

#include <systemc>
#include "usart.h"

void module_register() { GSC_MODULE_REGISTER_C(Usart); }
