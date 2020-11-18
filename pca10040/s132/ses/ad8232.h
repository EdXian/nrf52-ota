#ifndef AD8232_H
#define AD8232_H

#include <stdint.h>

#define AD8232_SDH_PIN_NUM          4
#define AD8232_LOD_PLUS_PIN_NUM     3
#define AD8232_LOD_MINUS_PIN_NUM    2

#define LOD_PLUS_POS                (15)
#define LOD_MINUS_POS               (14)
#define LOD_PLUS_MASK               (1<<15)
#define LOD_MINUS_MASK              (1<<14)

typedef struct _ad8232_t {
  void (*init)(void);
  void (*update_lod)(void);
  uint16_t lod_status;
}ad8232_t;


#endif /* AD8232_H */
