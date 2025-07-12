/*
 * commands.h
 *
 *  Created on: Jul 4, 2025
 *      Author: luisduquefranco
 */

#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_

void Cmd_HandleLEDDelayCmd(const char *arg);
void Cmd_HandleSampleFreqCmd(const char *arg);
void Cmd_HandleRGBCmd(const char *arg);
void Cmd_HandleFFTSizeCmd(const char *arg);
void Cmd_HandleStatusCmd(void);
void Cmd_HandlePrintADC(void);
void Cmd_HandleFreqDisplayCmd(void);
void Cmd_HandlePrintFFT(void);
void Cmd_HandleClearCmd(void);
void Cmd_HandleHelpCmd(void);


#endif /* INC_COMMANDS_H_ */
