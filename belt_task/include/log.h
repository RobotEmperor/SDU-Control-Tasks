#ifndef LOG_H_
#define LOG_H_

#include <cinttypes>

#define COLOR_RESET "\033[0m"
#define COLOR_BLACK "\033[30m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_WHITE "\033[37m"
#define COLOR_BLACK_BOLD "\033[1m\033[30m"
#define COLOR_RED_BOLD "\033[1m\033[31m"
#define COLOR_GREEN_BOLD "\033[1m\033[32m"
#define COLOR_YELLOW_BOLD "\033[1m\033[33m"
#define COLOR_BLUE_BOLD "\033[1m\033[34m"
#define COLOR_MAGENTA_BOLD "\033[1m\033[35m"
#define COLOR_CYAN_BOLD "\033[1m\033[36m"
#define COLOR_WHITE_BOLD "\033[1m\033[37m"

/* FOREGROUND */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x) "\x1B[1m" x RST
#define UNDL(x) "\x1B[4m" x RST

#define LOG_DEBUG(format, ...) printf("\x1B[35m[DEBUG]: " format "\n\x1B[0m", ##__VA_ARGS__)
#define LOG_WARN(format, ...) printf("\x1B[33m[WARNING]: " format "\n\x1B[0m", ##__VA_ARGS__)
#define LOG_INFO(format, ...) printf("\x1B[34m[INFO]: " format "\n\x1B[0m", ##__VA_ARGS__)
#define LOG_ERROR(format, ...) printf("\x1B[31m[ERROR]: " format "\n\x1B[0m", ##__VA_ARGS__)
#define LOG_FATAL(format, ...) printf("\x1B[31m[FATAL]: " format "\n\x1B[0m", ##__VA_ARGS__))

#endif  /* LOG_H_ */
