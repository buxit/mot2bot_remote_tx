void lcdml_menu_clear()
{
  lcd.clear();
  lcd.setCursor(0, 0);
}

void lcdml_menu_display()
{
  if (LCDML.DISP_checkMenuUpdate()) {
    LCDML.DISP_clear();

    // HACK: skip layer 0
    if (LCDML.MENU_getLayer() == 0)
      return;

    char content_text[_LCDML_DISP_cols];
    // menu element object
    LCDMenuLib2_menu *tmp;

    uint8_t i = LCDML.MENU_getScroll();
    uint8_t maxi = _LCDML_DISP_rows + i;
    uint8_t n = 0;

    // check if this element has children
    if (tmp = LCDML.MENU_getObj()->getChild(LCDML.MENU_getScroll()))
    {
      // loop to display lines
      do
      {
        // check if a menu element has a condetion and if the condetion be true
        if (tmp->checkCondetion())
        {
          // check the type off a menu element
          if (tmp->checkType_menu() == true)
          {
            // display normal content
            LCDML_getContent(content_text, tmp->getID());
            lcd.setCursor(1, n);
            lcd.print(content_text);
            if (tmp->getChild(0)) {
              lcd.setCursor(14, n);
              lcd.write((uint8_t)5);
            }
            Serial.println(content_text);
          }
          else
          {
            if (tmp->checkType_dynParam()) {
              tmp->callback(n);
            }
          }
          // increment some values
          i++;
          n++;
        }
        // try to go to the next sibling and check the number of displayed rows
      } while (((tmp = tmp->getSibling(1)) != NULL) && (i < maxi));
    }
  }

  if (LCDML.DISP_checkMenuCursorUpdate())
  {
    // init vars
    uint8_t n_max             = (LCDML.MENU_getChilds() >= _LCDML_DISP_rows) ? _LCDML_DISP_rows : (LCDML.MENU_getChilds());
    uint8_t scrollbar_min     = 0;
    uint8_t scrollbar_max     = LCDML.MENU_getChilds();
    uint8_t scrollbar_cur_pos = LCDML.MENU_getCursorPosAbs();

    float scroll_pos_float    = (((float)n_max) / (scrollbar_max - 0.99)) * (scrollbar_cur_pos); /* -0.99 so we never reach the upper bound */
    uint8_t scroll_pos_int    = scroll_pos_float;
    uint8_t scroll_pos_rel    = (uint8_t)((scroll_pos_float - scroll_pos_int) * 4.0); /* 4 scrollbar symbols */
    /*
        Serial.print("scrollbar_cur_pos=");
        Serial.print(scrollbar_cur_pos);
        Serial.print(", scrollbar_max=");
        Serial.print(scrollbar_max);
        Serial.print(", int=");
        Serial.print(scroll_pos_int);
        Serial.print(", rel=");
        Serial.print(scroll_pos_rel);
        Serial.print(", scroll_pos=");
        Serial.println(scroll_pos_float);
    */
    // display rows
    for (uint8_t n = 0; n < n_max; n++)
    {
      //set cursor
      lcd.setCursor(0, n);

      //set cursor char
      if (n == LCDML.MENU_getCursorPos()) {
        lcd.write(_LCDML_DISP_cfg_cursor);
      } else {
        lcd.write(' ');
      }

      // delete or reset scrollbar
      if (_LCDML_DISP_cfg_scrollbar == 1) {
        if (scrollbar_max > n_max) {
          lcd.setCursor((_LCDML_DISP_cols - 1), n);
          lcd.write((uint8_t)0);
        }
        else {
          lcd.setCursor((_LCDML_DISP_cols - 1), n);
          lcd.print(' ');
        }
      }
    }

    // display scrollbar
    if (_LCDML_DISP_cfg_scrollbar == 1) {
      if (scrollbar_max > n_max) {
        lcd.setCursor((_LCDML_DISP_cols - 1), scroll_pos_int);
        lcd.write((uint8_t)(scroll_pos_rel) + 1);
      }
    }
  }
}
