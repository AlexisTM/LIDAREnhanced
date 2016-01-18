/* 
 *  KFilter.h is an example of implementation of a outlier detection algorithm 
 *  
 *  It also converts integers to doubles
 *  
 * Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 * This software may be distributed and modified under the terms of the GNU
 * General Public License version 2 (GPL2) as published by the Free Software
 * Foundation and appearing in the file GPL2.TXT included in the packaging of
 * this file. Please note that GPL2 Section 2[b] requires that all works based
 * on this software must also be made publicly available under the terms of
 * the GPL2 ("Copyleft").
 * 
 * 
 * Contact information
 * -------------------
 * Alexis Paques
 * e-mail   :  alexis.paques@gmail.com
 */

#ifndef _KFilter_h
#define _KFilter_h
class KFilter {
  public:
/*******************************************************************************
  Constructor
*******************************************************************************/
    KFilter() {};
    void begin(double _initialValue, double _initialSpeed = 0, double _maximalSpeed = 100, double _linearity = 1, double _offset = 0);
    //double filter(double actualPosition);
    double filter(double actualPosition, bool * isItAnOutlier);
    bool isOutlier(double actualPosition, double predictedPosition, double dt);
    inline double correct_offset(double val){
      return val*factors[0] + factors[1];
    };

    int getOutlierCount(){ return outlierCount; };
    int resetOutlierCount(){ outlierCount = 0; };

  private:
    int outlierCount = 0;
    long lastTimestamp;
    double lastSpeed, maximalSpeed, lastPosition;

    double factors[2];
};

#endif
