/**
 * SerialCommand - A Wiring/Arduino library to tokenize and parse commands
 * received over a serial port.
 * 
 * Copyright (C) 2012 Stefan Rado
 * Copyright (C) 2011 Steven Cogswell <steven.cogswell@gmail.com>
 *                    http://husks.wordpress.com
 * 
 * Version 20120522
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "SerialCommand.h"
#include "Globals.h"
#include "Util.h"

/**
 * Constructor makes sure some things are set.
 */
SerialCommand::SerialCommand(const  Command* commands, uint8_t commandCount, void (*defaultHandler)(const char *)) :
		commands(commands),
		commandCount(commandCount),
		defaultHandler(NULL),
		wasCRLF(false), // default terminator for commands, newline character
		last(NULL) {
	strcpy(delim, " "); // strtok_r needs a null-terminated string
	clearBuffer();
}

/**
 * Adds a "command" and a handler function to the list of available commands.
 * This is used for matching a found token in the buffer, and gives the pointer
 * to the handler function to deal with it.
 * dongfang TODO: Make static.
void SerialCommand::addCommand(const char *command, void (*function)()) {
	commandList = (SerialCommandCallback *) realloc(commandList, (commandCount + 1) * sizeof(SerialCommandCallback));
	strncpy(commandList[commandCount].command, command, SERIALCOMMAND_MAXCOMMANDLENGTH);
	commandList[commandCount].function = function;
	commandCount++;
}
 */

/**
 * This sets up a handler to be called in the event that the receveived command string
 * isn't in the list of commands.
void SerialCommand::setDefaultHandler(void (*function)(const char *)) {
	defaultHandler = function;
}
 */

/**
 * This checks the Serial stream for characters, and assembles them into a buffer.
 * When the terminator character (default '\n') is seen, it starts parsing the
 * buffer for a prefix command, and calls handlers setup by addCommand() member
 */
void SerialCommand::readSerial() {
	while (serial0.available() > 0) {
		char inChar = getchar(); // Read single available character, there may be more waiting
		if (inChar == '\r' || inChar == '\n') { // Check for the terminator (default '\r') meaning end of command
			if (wasCRLF) continue;
			wasCRLF = true;
			char *command = strtok_r(buffer, delim, &last); // Search for command at start of buffer
			if (command != NULL) {
				bool matched = false;
				// Compare the found command against the list of known commands for a match
				for (int j = 0; command[j] != '\0'; j++) // as no strnicmp exists ...
					command[j] = (char) tolower(command[j]);
				for (int i = 0; i < commandCount; i++) {
					PGM_P *blarh = (PGM_P*)pgm_read_word(commands[i].text);
					PGM_P fims = (PGM_P)pgm_read_word(blarh);
					printf_P(PSTR("Trying %s against %S\r\n"), command, fims);
					if (strncmp_P(command, fims, SERIALCOMMAND_MAXCOMMANDLENGTH) == 0) {
						// Execute the stored handler function for the command
						(*commands[i].function)();
						matched = true;
						break;
					}
				}
				if (!matched && (defaultHandler != NULL)) {
					(*defaultHandler)(command);
				}
			} else printf_P(PSTR("null command\r\n"));
			clearBuffer();
		} else {
			wasCRLF = false;
			if (isprint(inChar)) { // Only printable characters into the buffer
				if (bufPos < SERIALCOMMAND_BUFFER) {
					buffer[bufPos++] = inChar; // Put character into buffer
					buffer[bufPos] = '\0'; // Null terminate
				}
			}
		}
	}
}

/*
 * Clear the input buffer.
 */
void SerialCommand::clearBuffer() {
	buffer[0] = '\0';
	bufPos = 0;
}

/**
 * Retrieve the next token ("word" or "argument") from the command buffer.
 * Returns NULL if no more tokens exist.
 */
char *SerialCommand::next() {
	return strtok_r(NULL, delim, &last);
}
