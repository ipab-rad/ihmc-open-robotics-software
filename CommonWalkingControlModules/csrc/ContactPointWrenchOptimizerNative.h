/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative */

#ifndef _Included_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
#define _Included_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
#ifdef __cplusplus
extern "C" {
#endif
#undef us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_NUMBER_OF_POINTS_PER_CONTACT
#define us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_NUMBER_OF_POINTS_PER_CONTACT 4L
#undef us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_NUMBER_OF_SUPPORT_VECTORS
#define us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_NUMBER_OF_SUPPORT_VECTORS 4L
#undef us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_MAX_NUMBER_OF_CONTACTS
#define us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_MAX_NUMBER_OF_CONTACTS 4L
#undef us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_WRENCH_LENGTH
#define us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_WRENCH_LENGTH 6L
/*
 * Class:     us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
 * Method:    initialize
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_initialize
  (JNIEnv *, jclass);

/*
 * Class:     us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
 * Method:    getABuffer
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getABuffer
  (JNIEnv *, jclass);

/*
 * Class:     us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
 * Method:    getWBuffer
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getWBuffer
  (JNIEnv *, jclass);

/*
 * Class:     us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
 * Method:    getCBuffer
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getCBuffer
  (JNIEnv *, jclass);

/*
 * Class:     us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
 * Method:    getRhoMinBuffer
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getRhoMinBuffer
  (JNIEnv *, jclass);

/*
 * Class:     us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
 * Method:    getRhoBuffer
 * Signature: ()Ljava/nio/ByteBuffer;
 */
JNIEXPORT jobject JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getRhoBuffer
  (JNIEnv *, jclass);

/*
 * Class:     us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
 * Method:    solveNative
 * Signature: (D)I
 */
JNIEXPORT jint JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_solveNative
  (JNIEnv *, jclass, jdouble);

/*
 * Class:     us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative
 * Method:    getOptValNative
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_us_ihmc_commonWalkingControlModules_controlModules_nativeOptimization_ContactPointWrenchOptimizerNative_getOptValNative
  (JNIEnv *, jclass);

#ifdef __cplusplus
}
#endif
#endif
