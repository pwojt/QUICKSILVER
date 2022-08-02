#pragma once

void rgb_init();
void rgb_send(int data);

void rgb_dma_start();
void rgb_initIO();
void rgb_initTIM();
void rgb_initNVIC();
void rgb_initDMA();