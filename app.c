/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "em_common.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_board_control.h"
#include "em_types.h"
#include "dmd.h"
#include "glib.h"
#include <math.h>
#include <stdio.h>
#define M_PI    3.14159265358979323846

enum { IDLE, GET_SERVICE, GET_CHARACTERISTIC, GET_CURRENT_TIME, DONE } state;
uint32_t service_handle;
uint16_t current_time_handle;
GLIB_Context_t lcd;
sl_sleeptimer_date_t date;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

//#ifdef SL_CATALOG_PRINTF_PRESENT
#ifdef SL_CATALOG_IOSTREAM_USART_PRESENT
#define assertzero(TYPE,X) do { \
  TYPE rc = X; \
  if(rc) { \
    printf("%s:%d: assertion failed: %s\nrc: 0x%lx\n",__FILE__,__LINE__,#X,rc); \
    while(1); \
  } \
} while(0)
#define assert(X) do { \
    if(!(X)) { \
        printf("%s:%d: assertion failed: 0 == %s\n",__FILE__,__LINE__,#X); \
        while(1); \
    } \
} while(0)
#else
#define assertzero(TYPE,X) do { \
    if(X) { \
        while(1); \
    } \
} while(0)
#define assert(X) do { \
    if(!(X)) { \
        while(1); \
    } \
} while(0)
#endif
/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)
{
#ifdef SL_CATALOG_IOSTREAM_USART_PRESENT
  sl_board_enable_vcom();
#endif
  sl_board_enable_display();
  assertzero(EMSTATUS,DMD_init(NULL));
  assertzero(EMSTATUS,GLIB_contextInit(&lcd));
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}


void draw_hand(float angle, float length, float width, int invert) {
  float dangle = 0.75*width/length;
  int32_t hand[8] = {
      64, 64,
      64+.75*length*sin(angle-dangle), 64+0.75*length*cos(angle-dangle),
      64+length*sin(angle), 64+length*cos(angle),
      64+.75*length*sin(angle+dangle), 64+0.75*length*cos(angle+dangle)
  };
  lcd.foregroundColor = (invert)?Black:White;
  lcd.backgroundColor = (invert)?White:Black;
  GLIB_drawPolygonFilled(&lcd,4,hand);
}

void update_clock() {
  lcd.foregroundColor = White;
  lcd.backgroundColor = Black;
  assertzero(EMSTATUS,GLIB_clear(&lcd));
  assertzero(EMSTATUS,GLIB_drawCircleFilled(&lcd,  64, 64, 63));
  lcd.foregroundColor = Black;
  lcd.backgroundColor = White;
  assertzero(EMSTATUS,GLIB_drawCircleFilled(&lcd,  64, 64, 56));
  lcd.foregroundColor = White;
  lcd.backgroundColor = Black;
  assertzero(EMSTATUS,GLIB_setFont(&lcd, (void*)&GLIB_FontNormal8x8));
  for(int i = 0; i < 12; i++) {
      double angle = (6+i)*M_PI/6;
      int x = 64+48*sin(angle);
      int y = 64+48*cos(angle);
      char buf[3];
      sprintf(buf,"%d",12-i);
      assertzero(EMSTATUS,GLIB_drawString(&lcd,buf,strlen(buf),x-4*strlen(buf),y-2,true));
  }
  assertzero(sl_status_t,sl_sleeptimer_get_datetime(&date));
  draw_hand((6-(date.hour%12+date.min/60.))*M_PI/6,24,9,0);
  draw_hand((30-(date.min+date.sec/60.))*M_PI/30,40,9,1);
  draw_hand((30-(date.min+date.sec/60.))*M_PI/30,40,5,0);
  draw_hand((30-date.sec)*M_PI/30,52,7,1);
  draw_hand((30-date.sec)*M_PI/30,52,3,0);
  assertzero(EMSTATUS,DMD_updateDisplay());
}

void set_time(uint8_t len, uint8_t *data) {
  struct __attribute__((packed)) sig_current_time {
    struct __attribute__((packed)) {
      struct __attribute__((packed)) {
        struct __attribute__((packed)) {
          uint16_t year;
          uint8_t month;
          uint8_t day;
          uint8_t hours;
          uint8_t minutes;
          uint8_t seconds;
        } date_time;
        struct __attribute__((packed)) {
          uint8_t day;
        } day_of_week;
      }day_date_time;
      uint8_t fractions;
    } exact_time_256;
    uint8_t adjust_reason;
  } *p = (void*) data;
  assert(sizeof(struct sig_current_time) == len);
#ifdef SL_CATALOG_IOSTREAM_USART_PRESENT
#define P(X) printf(#X ": %d\n", X)
#define Pl(X) printf(#X ": %ld\n", X)
  P(sizeof(struct sig_current_time));
  P(p->exact_time_256.day_date_time.date_time.year);
  P(p->exact_time_256.day_date_time.date_time.month);
  P(p->exact_time_256.day_date_time.date_time.day);
  P(p->exact_time_256.day_date_time.date_time.hours);
  P(p->exact_time_256.day_date_time.date_time.minutes);
  P(p->exact_time_256.day_date_time.date_time.seconds);
  P(p->exact_time_256.day_date_time.day_of_week.day);
  P(p->exact_time_256.fractions);
#endif
  assertzero(sl_status_t,sl_sleeptimer_build_datetime(&date,
      p->exact_time_256.day_date_time.date_time.year,
      p->exact_time_256.day_date_time.date_time.month,
      p->exact_time_256.day_date_time.date_time.day,
      p->exact_time_256.day_date_time.date_time.hours,
      p->exact_time_256.day_date_time.date_time.minutes,
      p->exact_time_256.day_date_time.date_time.seconds,
      0));
#ifdef SL_CATALOG_IOSTREAM_USART_PRESENT
  Pl(date.time_zone);
  P(date.day_of_year);
  P(date.day_of_week);
  P(date.year);
  P(date.month);
  P(date.month_day);
  P(date.hour);
  P(date.min);
  P(date.sec);
#undef P
#undef Pl
#endif
  assertzero(sl_status_t,sl_sleeptimer_set_datetime(&date));
}

void display_passkey(uint32_t passkey) {
  char buf[7];
  lcd.foregroundColor = Black;
  lcd.backgroundColor = White;
  assertzero(EMSTATUS,GLIB_clear(&lcd));
  assertzero(EMSTATUS,GLIB_setFont(&lcd, (void*)&GLIB_FontNormal8x8));
  assertzero(EMSTATUS,GLIB_drawString(&lcd,"ENTER",5,64-5*8/2,20,true));
  assertzero(EMSTATUS,GLIB_drawString(&lcd,"PASSCODE",8,64-8*8/2,32,true));
  assertzero(EMSTATUS,GLIB_drawString(&lcd,"ON OTHER",8,64-8*8/2,44,true));
  assertzero(EMSTATUS,GLIB_drawString(&lcd,"DEVICE",6,64-6*8/2,56,true));
  sprintf(buf,"%06ld",passkey);
  assertzero(EMSTATUS,GLIB_setFont(&lcd, (void*)&GLIB_FontNumber16x20));
  assertzero(EMSTATUS,GLIB_drawString(&lcd,buf,6,64-3*16,64+10,true));
  assertzero(EMSTATUS,DMD_updateDisplay());
}


/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      state = IDLE;
      sl_bt_sm_set_bondable_mode(1);
      sl_bt_sm_configure(7, sm_io_capability_displayonly);
      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n",
                    (int)sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n",
                    (int)sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      break;

    case sl_bt_evt_system_soft_timer_id:
      update_clock();
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      sl_bt_sm_increase_security(evt->data.evt_connection_opened.connection);
      break;

    case sl_bt_evt_sm_passkey_display_id:
      display_passkey(evt->data.evt_sm_passkey_display.passkey);
      break;

    case sl_bt_evt_connection_parameters_id:
      if(3 == evt->data.evt_connection_parameters.security_mode) {
          state = GET_SERVICE;
          service_handle = 0;
          sl_bt_gatt_discover_primary_services_by_uuid(evt->data.evt_connection_parameters.connection,  2, (uint8_t*)"\x05\x18");
      }
      break;

    case sl_bt_evt_gatt_service_id:
#define ED evt->data.evt_gatt_service
      if(2 != ED.uuid.len) break;
      if(0x05 != ED.uuid.data[0]) break;
      if(0x18 != ED.uuid.data[1]) break;
      service_handle = ED.service;
      break;
#undef ED

    case sl_bt_evt_gatt_characteristic_id:
#define ED evt->data.evt_gatt_characteristic
      if(2 != ED.uuid.len) break;
      if(0x2b != ED.uuid.data[0]) break;
      if(0x2a != ED.uuid.data[1]) break;
      current_time_handle = ED.characteristic;
      break;
#undef ED

    case sl_bt_evt_gatt_characteristic_value_id:
#define ED evt->data.evt_gatt_characteristic_value
      if(ED.characteristic == current_time_handle) {
          set_time(ED.value.len,ED.value.data);
          sl_bt_system_set_soft_timer(1<<15, 0, 0); // start timer to update LCD display
      }
      break;
#undef ED

    case sl_bt_evt_gatt_procedure_completed_id:
#define ED evt->data.evt_gatt_procedure_completed
      if(ED.result) break;
      switch(state) {
        case GET_SERVICE:
          if(service_handle) {
              state = GET_CHARACTERISTIC;
              current_time_handle = 0;
              sl_bt_gatt_discover_characteristics_by_uuid(ED.connection, service_handle, 2, (uint8_t*)"\x2b\x2a");
          }
          break;
        case GET_CHARACTERISTIC:
          if(current_time_handle) {
              state = GET_CURRENT_TIME;
              sl_bt_gatt_read_characteristic_value(ED.connection, current_time_handle);
          }
          break;
        case GET_CURRENT_TIME:
          state = DONE;
          break;
        case DONE:
          assert(NULL == "gatt_procedure_completed event in DONE state");
          break;
        case IDLE:
          assert(NULL == "gatt_procedure_completed event in IDLE state");
          break;
      }
      break;
#undef ED

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      break;

    case sl_bt_evt_gatt_server_attribute_value_id:
#define ED evt->data.evt_gatt_server_attribute_value
      date.hour = ED.value.data[0];
      sl_sleeptimer_set_datetime(&date);
      break;
#undef ED
    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
