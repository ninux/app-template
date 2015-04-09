/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)

#define BORDER sizeof(GaussFilter - 1)

const int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT/2;

const int GaussFilter[] = {82, 72, 50, 27, 11, 4, 1};

int TextColor;

int avgDxy[3][IMG_SIZE];
int helpBuf[IMG_SIZE];

void CalcDeriv(void);
void AvgDeriv(int index);

void ResetProcess()
{
	//called when "reset" button is pressed
	if(TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}

void ProcessFrame()
{
	uint32 t1, t2;
	char Text[] = "hallo world";
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for time measurement
		t1 = OscSupCycGet();
		//example for copying sensor image to background image
		memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG], IMG_SIZE);
		//example for time measurement
		t2 = OscSupCycGet();

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));

		//example for drawing output
		//draw line
		DrawLine(10, 100, 200, 20, RED);
		//draw open rectangle
		DrawBoundingBox(20, 10, 50, 40, false, GREEN);
		//draw filled rectangle
		DrawBoundingBox(80, 100, 110, 120, true, BLUE);
		DrawString(200, 200, strlen(Text), TINY, TextColor, Text);
	}
	CalcDeriv();
}


void CalcDeriv()
{
	int c, r;

	for(r = nc; r < nr*nc-nc; r+= nc) {	/* skip first and last line */
		for(c = 1; c < nc-1; c++) {
			/* do pointer arithmetics with respect to center pixel
			 * location */
			unsigned char* p = &data.u8TempImage[SENSORIMG][r+c];

			/* implement Sobel filter */
			int dx = -(int) *(p-nc-1) + (int) *(p-nc+1)
				-2* (int) *(p-1) + 2* (int) *(p+1)
				-(int) *(p+nc-1) + (int) *(p+nc+1);

			int dy = -(int) *(p-nc-1) -2* (int) *(p-nc) -(int) *(p-nc+1)
				+ (int) *(p+nc-1) + 2* (int) *(p+nc) + (int) *(p+nc+1);

			avgDxy[0][r+c] = dx*dx;
			avgDxy[1][r+c] = dy*dy;
			avgDxy[2][r+c] = dx*dy;

			data.u8TempImage[BACKGROUND][r+c] = (uint8)MIN(255, MAX(0, 128+dx));
			data.u8TempImage[THRESHOLD][r+c] = (uint8)MIN(255, MAX(0, 128+dy));
		}
	}
}

void AvgDeriv(int index)
{
	int c, r, s;

	for(r = nc; r < nc*nc-nc; r += nc) {
		for(c = BORDER+1; c < nc-(BORDER+1); c++) {
			int* p = &avgDxy[index][r+c];

			/*
			int sx = (*(p))*GaussFilter[0]
				+ (*(p-1) + *(p+1))*GaussFilter[1]
				+ (*(p-3) + *(p+3))*GaussFilter[2]
				+ (*(p-4) + *(p+4))*GaussFilter[3]
				+ (*(p-5) + *(p+5))*GaussFilter[4]
				+ (*(p-6) + *(p+6))*GaussFilter[5]
				+ (*(p-7) + *(p+7))*GaussFilter[6];
			*/

			int sx = (*(p))*GaussFilter[0];

			for(s = 1; s < 7; s++) {
				sx += (*(p-s) + *(p+s))*GaussFilter[s];
			}

			helpBuf[r+c] = (sx >> 8);
		}
	}

	for(r = nc; r < nc*nc-nc; r += nc) {
		for(c = BORDER+1; c < nc-(BORDER+1); c++) {
			int* p = &helpBuf[r+c];

			/*
			int sx = (*(p))*GaussFilter[0]
				+ (*(p-1) + *(p+1))*GaussFilter[1]
				+ (*(p-3) + *(p+3))*GaussFilter[2]
				+ (*(p-4) + *(p+4))*GaussFilter[3]
				+ (*(p-5) + *(p+5))*GaussFilter[4]
				+ (*(p-6) + *(p+6))*GaussFilter[5]
				+ (*(p-7) + *(p+7))*GaussFilter[6];
			*/

			int sy = (*(p))*GaussFilter[0];

			for(s = 1; s < 7; s++) {
				sy += (*(p-(s*nc)) + *(p+(s*nc)))*GaussFilter[s];
			}

			avgDxy[index][r+c] = (sy >> 8);
		}
	}
}
