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

#define CORNER_PARAM_DEFAULT 5

const int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT/2;

#define SPEED_OPTIMIZING 1

#ifndef SPEED_OPTIMIZING
const int GaussFilter[] = {82, 72, 50, 27, 11, 4, 1};
#else
const int GaussFilter[] = {1, 4, 8, 32, 64, 64, 128};
#endif

int TextColor;

int avgDxy[3][IMG_SIZE];
int helpBuf[IMG_SIZE];
int WAC[IMG_SIZE];

void CalcDeriv(void);
void AvgDeriv(int index);
void CalcWAC(int k);
void LocateMax(void);
void DrawBoxes(void);

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
	// char Text[] = "hallo world";
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for time measurement
		t1 = OscSupCycGet();
		//example for copying sensor image to background image
		// memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG], IMG_SIZE);
		//example for time measurement
		CalcDeriv();
		CalcWAC(CORNER_PARAM_DEFAULT);
		LocateMax();
		DrawBoxes();
		t2 = OscSupCycGet();

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));

		//example for drawing output
		//draw line
		//DrawLine(10, 100, 200, 20, RED);
		//draw open rectangle
		//DrawBoundingBox(20, 10, 50, 40, false, GREEN);
		//draw filled rectangle
		//DrawBoundingBox(80, 100, 110, 120, true, BLUE);
		//DrawString(200, 200, strlen(Text), TINY, TextColor, Text);
	}
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

#ifdef TEST_DERIVATES
			data.u8TempImage[BACKGROUND][r+c] = (uint8)MIN(255, MAX(0, 128+dx));
			data.u8TempImage[THRESHOLD][r+c] = (uint8)MIN(255, MAX(0, 128+dy));
#endif
		}
	}
}

void AvgDeriv(int index)
{
	int c, r;
#ifndef SPEED_OPTIMIZING
	s;
#endif

	for(r = nc; r < nr*nc-nc; r += nc) {
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

#ifndef SPEED_OPTIMIZING
			int sx = (*(p))*GaussFilter[0];

			for(s = 1; s < 7; s++) {
				sx += (*(p-s) + *(p+s))*GaussFilter[s];
			}
#else
			int sx =  ((*(p-6) + *(p+6)))
				+ ((*(p-5) + *(p+5)) << 2)
				+ ((*(p-4) + *(p+4)) << 3)
				+ ((*(p-3) + *(p+3)) << 5)
				+ ((*(p-2) + *(p+2)) << 6)
				+ ((*(p-1) + *(p+1)) << 6)
				+ ((*p << 7));
#endif
			helpBuf[r+c] = (sx >> 8);
		}
	}

	for(r = nc; r < nr*nc-nc; r += nc) {
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

#ifndef SPEED_OPTIMIZING
			int sy = (*(p))*GaussFilter[0];

			for(s = 1; s < 7; s++) {
				sy += (*(p-(s*nc)) + *(p+(s*nc)))*GaussFilter[s];
			}
#else
			int sy =  ((*(p-6) + *(p+6)))
				+ ((*(p-5) + *(p+5)) << 2)
				+ ((*(p-4) + *(p+4)) << 3)
				+ ((*(p-3) + *(p+3)) << 5)
				+ ((*(p-2) + *(p+2)) << 6)
				+ ((*(p-1) + *(p+1)) << 6)
				+ ((*p << 7));
#endif
			avgDxy[index][r+c] = (sy >> 8);
#ifdef TEST_GAUSSFILTER
			data.u8TempImage[BACKGROUND][r+c] =
				MAX(0,MIN(255,(avgDxy[2][r+c] >> 7)));
#endif
		}
	}
}

void CalcWAC(int k)
{
	int c, r = 0;
	int Ix2, Iy2, Ixy = 0;

	AvgDeriv(0);
	AvgDeriv(1);
	AvgDeriv(2);

	for(c = nc*(BORDER+1); r < (nr*nc - nc*(BORDER+1)); r += nc) {
		for(c = BORDER+1; c < nc-(BORDER+1); c++) {
			/* scale data to prevent overflow */
			Ix2 = avgDxy[0][r+c] >> 7;
			Iy2 = avgDxy[1][r+c] >> 7;
			Ixy = avgDxy[2][r+c] >> 7;

			avgDxy[2][r+c] = (Ix2*Iy2 - (Ixy^2))
				- ((k*((Ix2 + Iy2)^2)) >> 7);
#ifdef TEST_WAC
			data.u8TempImage[BACKGROUND][r+c] =
				MAX(0,MIN(255,(avgDxy[2][r+c] >> 7)));
#endif
		}
	}
}

void LocateMax(void)
{
	int c, r = 0;

	for(r = nc*(BORDER+1); r < (nr*nc - nc*(BORDER+1)); r += nc) {
		for(c = BORDER+1; c < nc-(BORDER+1); c++) {
			int *pos = &avgDxy[2][r+c];	// current position
			int val = avgDxy[2][r+c];	// current value
			int sc, sr = 0;			// scope column, row

			for(sc = -6; sc < 7; sc++) {
				for(sr = -6; sr < 7; sr++) {
					if (val < *((pos+sr)+(nc*sc))) {
						// is not a maximum -> set 0
						avgDxy[0][r+c] = 0;
						// break the loop
						sr = sc = 8;
					} else {
						// possible maximum -> keep
						avgDxy[0][r+c] = val;
					}
				}
			}
#ifdef TEST_LOCATE_MAX
			data.u8TempImage[BACKGROUND][r+c] =
				MAX(0,MIN(255,(avgDxy[0][r+c]>>7)));
#endif
		}
	}
}

void DrawBoxes(void)
{
	int c, r = 0;
	int SizeBox = 5;

	for(r = nc*(BORDER+1); r < (nr*nc - nc*(BORDER+1)); r += nc) {
		for(c = BORDER+1; c < nc-(BORDER+1); c++) {
			if((avgDxy[0][r+c]>>7) > ((255*data.ipc.state.nThreshold)/100)) {
				avgDxy[1][r+c] = 255;
				DrawBoundingBox(c-SizeBox,
						(r/nc)-SizeBox,
						c+SizeBox,
						(r/nc)+SizeBox,
						false,
						RED);
				printf("POS: %i \t%i\n",
				       r/(nc),
				       c);
			} else {
				avgDxy[1][r+c] = 0;
			}
			data.u8TempImage[BACKGROUND][r+c] =
				 MAX(0,MIN(255,(avgDxy[1][r+c])));
		}
	}
}
