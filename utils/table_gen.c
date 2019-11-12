/*
 * Generates the lookup table used for generating the sine waveform.
 *
 * Copyright (c) 2019 Stephen Rhen
 *
 *   This file is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 3, or (at your option)
 *   any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TABLE_SIZE   1024

const char *filename = "sine_table.c";

int main(int agrc, char *argv[])
{
  FILE *file;
  int i;
  int cnt = 0;  /* number of entries written to teh current line. */
  double angle = 0.0;
  double s, value;

  if ((file = fopen(filename, "w")) == NULL) {
    fprintf(stderr, "Failed to open output file %s", filename);
    exit(EXIT_FAILURE);
  }

  fprintf(file, "/*\n");
  fprintf(file, " *  sine_table.c\n");
  fprintf(file, " *\n *    Generated by table_gen\n");
  fprintf(file, " */\n\n");
  fprintf(file, "#include <Arduino.h>\n\n");
  fprintf(file, "#define SINE_TABLE_SIZE %i\n\n", TABLE_SIZE);
  fprintf(file, "const long sine_table_size = SINE_TABLE_SIZE;\n\n");
  fprintf(file, "const uint8_t sine_table[SINE_TABLE_SIZE] PROGMEM =\n  {");
  
  for (i = 0; i < TABLE_SIZE; i++) {
    s = sin(angle);
    value = s * 127 + 128;
    fprintf(file, " %i, ", (int)value);
    if (++cnt == 8) {
      fprintf(file, "\n  ");
      cnt = 0;
    }
    angle += M_PI*2/TABLE_SIZE;
  }

  fprintf(file, "};\n\n");

  fclose(file);

  return 0;

}
