/*
 * cmd_def.h
 *
 *  Created on: 4 feb. 2023
 *      Author: ezequiel
 */

#ifndef INC_CMD_DEF_H_
#define INC_CMD_DEF_H_

#pragma once

/** Definición de OPCODES (códigos de operación de los telecomandos). */
enum opcode_index_e {
    /** Enviar un mensaje CAN.
     */
	OPCODE_SEND_CAN_MSG 					= 0x00,

	// END Opcodes específicos de la aplicación
	OPCODE_LAST
};


#endif /* INC_CMD_DEF_H_ */
