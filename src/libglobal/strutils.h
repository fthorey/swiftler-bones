#ifndef STRUTILS_H
# define STRUTILS_H

void reverse(char* s);
char* itoa(int n, char* s);
float atofl(const char* str);
char* fltoa(float f, char *s);
int atoi_eol(const char* str, char eol);
int atoi(const char* str);
int htoi_eol(const char* str, char eol);
int htoi(const char* str);
int xtoi_eol(const char* str, char eol);
int xtoi(const char* str);
char* trim_in_place(char* str);
int is_letter(char c);
int is_number(char c);
int is_space(char c);

#endif
