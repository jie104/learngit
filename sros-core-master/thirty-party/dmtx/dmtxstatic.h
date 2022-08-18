/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact: Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtxstatic.h
 * \brief extern header
 */
#include "dmtx.h"
#ifndef __DMTXSTATIC_H__
#define __DMTXSTATIC_H__

#define DmtxAlmostZero          0.000001
#define DmtxAlmostInfinity            -1

#define DmtxValueC40Latch            230
#define DmtxValueTextLatch           239
#define DmtxValueX12Latch            238
#define DmtxValueEdifactLatch        240
#define DmtxValueBase256Latch        231

#define DmtxValueCTXUnlatch   254
#define DmtxValueEdifactUnlatch       31

#define DmtxValueAsciiPad            129
#define DmtxValueAsciiUpperShift     235
#define DmtxValueCTXShift1      0
#define DmtxValueCTXShift2      1
#define DmtxValueCTXShift3      2
#define DmtxValueFNC1                232
#define DmtxValueStructuredAppend    233
#define DmtxValue05Macro             236
#define DmtxValue06Macro             237
#define DmtxValueECI                 241

#define DmtxC40TextBasicSet            0
#define DmtxC40TextShift1              1
#define DmtxC40TextShift2              2
#define DmtxC40TextShift3              3

#define DmtxUnlatchExplicit            0
#define DmtxUnlatchImplicit            1

#define DmtxChannelValid            0x00
#define DmtxChannelUnsupportedChar  0x01 << 0
#define DmtxChannelCannotUnlatch    0x01 << 1

#undef min
#define min(X,Y) (((X) < (Y)) ? (X) : (Y))

#undef max
#define max(X,Y) (((X) > (Y)) ? (X) : (Y))

typedef enum {
   DmtxEncodeNormal,  /* Use normal scheme behavior (e.g., ASCII auto) */
   DmtxEncodeCompact, /* Use only compact format within scheme */
   DmtxEncodeFull     /* Use only fully expanded format within scheme */
} DmtxEncodeOption;

typedef enum {
   DmtxRangeGood,
   DmtxRangeBad,
   DmtxRangeEnd
} DmtxRange;

typedef enum {
   DmtxEdgeTop               = 0x01 << 0,
   DmtxEdgeBottom            = 0x01 << 1,
   DmtxEdgeLeft              = 0x01 << 2,
   DmtxEdgeRight             = 0x01 << 3
} DmtxEdge;

typedef enum {
   DmtxMaskBit8              = 0x01 << 0,
   DmtxMaskBit7              = 0x01 << 1,
   DmtxMaskBit6              = 0x01 << 2,
   DmtxMaskBit5              = 0x01 << 3,
   DmtxMaskBit4              = 0x01 << 4,
   DmtxMaskBit3              = 0x01 << 5,
   DmtxMaskBit2              = 0x01 << 6,
   DmtxMaskBit1              = 0x01 << 7
} DmtxMaskBit;

/**
 * @struct DmtxFollow
 * @brief DmtxFollow
 */
typedef struct DmtxFollow_struct {
   unsigned char  *ptr;
   unsigned char   neighbor;
   int             step;
   DmtxPixelLoc    loc;
} DmtxFollow;

/**
 * @struct DmtxBresLine
 * @brief DmtxBresLine
 */
typedef struct DmtxBresLine_struct {
   int             xStep;
   int             yStep;
   int             xDelta;
   int             yDelta;
   int             steep;
   int             xOut;
   int             yOut;
   int             travel;
   int             outward;
   int             error;
   DmtxPixelLoc    loc;
   DmtxPixelLoc    loc0;
   DmtxPixelLoc    loc1;
} DmtxBresLine;

typedef struct C40TextState_struct {
   int             shift;
   DmtxBoolean     upperShift;
} C40TextState;

/* dmtxregion.c */
extern double RightAngleTrueness(DmtxVector2 c0, DmtxVector2 c1, DmtxVector2 c2, double angle);
extern DmtxPointFlow MatrixRegionSeekEdge(DmtxDecode *dec, DmtxPixelLoc loc0);
extern DmtxPassFail MatrixRegionOrientation(DmtxDecode *dec, DmtxRegion *reg, DmtxPointFlow flowBegin);
extern long DistanceSquared(DmtxPixelLoc a, DmtxPixelLoc b);
extern int ReadModuleColor(DmtxDecode *dec, DmtxRegion *reg, int symbolRow, int symbolCol, int sizeIdx, int colorPlane);

extern DmtxPassFail MatrixRegionFindSize(DmtxDecode *dec, DmtxRegion *reg);
extern int CountJumpTally(DmtxDecode *dec, DmtxRegion *reg, int xStart, int yStart, DmtxDirection dir);
extern DmtxPointFlow GetPointFlow(DmtxDecode *dec, int colorPlane, DmtxPixelLoc loc, int arrive);
extern DmtxPointFlow FindStrongestNeighbor(DmtxDecode *dec, DmtxPointFlow center, int sign);
extern DmtxFollow FollowSeek(DmtxDecode *dec, DmtxRegion *reg, int seek);
extern DmtxFollow FollowSeekLoc(DmtxDecode *dec, DmtxPixelLoc loc);
extern DmtxFollow FollowStep(DmtxDecode *dec, DmtxRegion *reg, DmtxFollow followBeg, int sign);
extern DmtxFollow FollowStep2(DmtxDecode *dec, DmtxFollow followBeg, int sign);
extern DmtxPassFail TrailBlazeContinuous(DmtxDecode *dec, DmtxRegion *reg, DmtxPointFlow flowBegin, int maxDiagonal);
extern int TrailBlazeGapped(DmtxDecode *dec, DmtxRegion *reg, DmtxBresLine line, int streamDir);
extern int TrailClear(DmtxDecode *dec, DmtxRegion *reg, int clearMask);
extern DmtxBestLine FindBestSolidLine(DmtxDecode *dec, DmtxRegion *reg, int step0, int step1, int streamDir, int houghAvoid);
extern DmtxBestLine FindBestSolidLine2(DmtxDecode *dec, DmtxPixelLoc loc0, int tripSteps, int sign, int houghAvoid);
extern DmtxPassFail FindTravelLimits(DmtxDecode *dec, DmtxRegion *reg, DmtxBestLine *line);
extern DmtxPassFail MatrixRegionAlignCalibEdge(DmtxDecode *dec, DmtxRegion *reg, int whichEdge);
extern DmtxBresLine BresLineInit(DmtxPixelLoc loc0, DmtxPixelLoc loc1, DmtxPixelLoc locInside);
extern DmtxPassFail BresLineGetStep(DmtxBresLine line, DmtxPixelLoc target, int *travel, int *outward);
extern DmtxPassFail BresLineStep(DmtxBresLine *line, int travel, int outward);
/*extern void WriteDiagnosticImage(DmtxDecode *dec, DmtxRegion *reg, char *imagePath);*/

/* dmtxdecode.c */
extern void TallyModuleJumps(DmtxDecode *dec, DmtxRegion *reg, int tally[][24], int xOrigin, int yOrigin, int mapWidth, int mapHeight, DmtxDirection dir);
extern DmtxPassFail PopulateArrayFromMatrix(DmtxDecode *dec, DmtxRegion *reg, DmtxMessage *msg);

/* dmtxdecodescheme.c */
extern void DecodeDataStream(DmtxMessage *msg, int sizeIdx, unsigned char *outputStart);
extern int GetEncodationScheme(unsigned char cw);
extern void PushOutputWord(DmtxMessage *msg, int value);
extern void PushOutputC40TextWord(DmtxMessage *msg, C40TextState *state, int value);
extern void PushOutputMacroHeader(DmtxMessage *msg, int macroType);
extern void PushOutputMacroTrailer(DmtxMessage *msg);
extern unsigned char *DecodeSchemeAscii(DmtxMessage *msg, unsigned char *ptr, unsigned char *dataEnd);
extern unsigned char *DecodeSchemeC40Text(DmtxMessage *msg, unsigned char *ptr, unsigned char *dataEnd, DmtxScheme encScheme);
extern unsigned char *DecodeSchemeX12(DmtxMessage *msg, unsigned char *ptr, unsigned char *dataEnd);
extern unsigned char *DecodeSchemeEdifact(DmtxMessage *msg, unsigned char *ptr, unsigned char *dataEnd);
extern unsigned char *DecodeSchemeBase256(DmtxMessage *msg, unsigned char *ptr, unsigned char *dataEnd);

/* dmtxencode.c */
extern void PrintPattern(DmtxEncode *encode);
extern int EncodeDataCodewords(DmtxByteList *input, DmtxByteList *output, int sizeIdxRequest, DmtxScheme scheme);

/* dmtxplacemod.c */
extern int ModulePlacementEcc200(unsigned char *modules, unsigned char *codewords, int sizeIdx, int moduleOnColor);
extern void PatternShapeStandard(unsigned char *modules, int mappingRows, int mappingCols, int row, int col, unsigned char *codeword, int moduleOnColor);
extern void PatternShapeSpecial1(unsigned char *modules, int mappingRows, int mappingCols, unsigned char *codeword, int moduleOnColor);
extern void PatternShapeSpecial2(unsigned char *modules, int mappingRows, int mappingCols, unsigned char *codeword, int moduleOnColor);
extern void PatternShapeSpecial3(unsigned char *modules, int mappingRows, int mappingCols, unsigned char *codeword, int moduleOnColor);
extern void PatternShapeSpecial4(unsigned char *modules, int mappingRows, int mappingCols, unsigned char *codeword, int moduleOnColor);
extern void PlaceModule(unsigned char *modules, int mappingRows, int mappingCols, int row, int col,
      unsigned char *codeword, int mask, int moduleOnColor);

/* dmtxreedsol.c */
extern DmtxPassFail RsEncode(DmtxMessage *message, int sizeIdx);
extern DmtxPassFail RsDecode(unsigned char *code, int sizeIdx, int fix);
extern DmtxPassFail RsGenPoly(DmtxByteList *gen, int errorWordCount);
extern DmtxBoolean RsComputeSyndromes(DmtxByteList *syn, const DmtxByteList *rec, int blockErrorWords);
extern DmtxBoolean RsFindErrorLocatorPoly(DmtxByteList *elp, const DmtxByteList *syn, int errorWordCount, int maxCorrectable);
extern DmtxBoolean RsFindErrorLocations(DmtxByteList *loc, const DmtxByteList *elp);
extern DmtxPassFail RsRepairErrors(DmtxByteList *rec, const DmtxByteList *loc, const DmtxByteList *elp, const DmtxByteList *syn);

/* dmtxscangrid.c */
extern DmtxScanGrid InitScanGrid(DmtxDecode *dec);
extern int PopGridLocation(DmtxScanGrid *grid, /*@out@*/ DmtxPixelLoc *locPtr);
extern int GetGridCoordinates(DmtxScanGrid *grid, /*@out@*/ DmtxPixelLoc *locPtr);
extern void SetDerivedFields(DmtxScanGrid *grid);

/* dmtxsymbol.c */
extern int FindSymbolSize(int dataWords, int sizeIdxRequest);

/* dmtximage.c */
extern int GetBitsPerPixel(int pack);

/* dmtxencodestream.c */
extern DmtxEncodeStream StreamInit(DmtxByteList *input, DmtxByteList *output);
extern void StreamCopy(DmtxEncodeStream *dst, DmtxEncodeStream *src);
extern void StreamMarkComplete(DmtxEncodeStream *stream, int sizeIdx);
extern void StreamMarkInvalid(DmtxEncodeStream *stream, int reasonIdx);
extern void StreamMarkFatal(DmtxEncodeStream *stream, int reasonIdx);
extern void StreamOutputChainAppend(DmtxEncodeStream *stream, DmtxByte value);
extern DmtxByte StreamOutputChainRemoveLast(DmtxEncodeStream *stream);
extern void StreamOutputSet(DmtxEncodeStream *stream, int index, DmtxByte value);
extern DmtxBoolean StreamInputHasNext(DmtxEncodeStream *stream);
extern DmtxByte StreamInputPeekNext(DmtxEncodeStream *stream);
extern DmtxByte StreamInputAdvanceNext(DmtxEncodeStream *stream);
extern void StreamInputAdvancePrev(DmtxEncodeStream *stream);

/* dmtxencodescheme.c */
extern int EncodeSingleScheme(DmtxByteList *input, DmtxByteList *output, int sizeIdxRequest, DmtxScheme scheme);
extern void EncodeNextChunk(DmtxEncodeStream *stream, int scheme, int subScheme, int sizeIdxRequest);
extern void EncodeChangeScheme(DmtxEncodeStream *stream, DmtxScheme targetScheme, int unlatchType);
extern int GetRemainingSymbolCapacity(int outputLength, int sizeIdx);

/* dmtxencodeoptimize.c */
extern int EncodeOptimizeBest(DmtxByteList *input, DmtxByteList *output, int sizeIdxRequest);
extern void StreamAdvanceFromBest(DmtxEncodeStream *streamNext,
      DmtxEncodeStream *streamList, int targeteState, int sizeIdxRequest);
extern void AdvanceAsciiCompact(DmtxEncodeStream *streamNext, DmtxEncodeStream *streamList,
      int state, int inputNext, int sizeIdxRequest);
extern void AdvanceCTX(DmtxEncodeStream *streamNext, DmtxEncodeStream *streamList,
      int state, int inputNext, int ctxValueCount, int sizeIdxRequest);
extern void AdvanceEdifact(DmtxEncodeStream *streamNext, DmtxEncodeStream *streamList,
      int state, int inputNext, int sizeIdxRequest);
extern int GetScheme(int state);
extern DmtxBoolean ValidStateSwitch(int fromState, int targetState);

/* dmtxencodeascii.c */
extern void EncodeNextChunkAscii(DmtxEncodeStream *stream, int option);
extern void AppendValueAscii(DmtxEncodeStream *stream, DmtxByte value);
extern void CompleteIfDoneAscii(DmtxEncodeStream *stream, int sizeIdxRequest);
extern void PadRemainingInAscii(DmtxEncodeStream *stream, int sizeIdx);
extern DmtxByteList EncodeTmpRemainingInAscii(DmtxEncodeStream *stream, DmtxByte *storage, int capacity, DmtxPassFail *passFail);
extern DmtxByte Randomize253State(DmtxByte cwValue, int cwPosition);

/* dmtxencodec40textx12.c */
extern void EncodeNextChunkCTX(DmtxEncodeStream *stream, int sizeIdxRequest);
extern void AppendValuesCTX(DmtxEncodeStream *stream, DmtxByteList *valueList);
extern void AppendUnlatchCTX(DmtxEncodeStream *stream);
extern void CompleteIfDoneCTX(DmtxEncodeStream *stream, int sizeIdxRequest);
extern void CompletePartialC40Text(DmtxEncodeStream *stream, DmtxByteList *valueList, int sizeIdxRequest);
extern void CompletePartialX12(DmtxEncodeStream *stream, DmtxByteList *valueList, int sizeIdxRequest);
extern DmtxBoolean PartialX12ChunkRemains(DmtxEncodeStream *stream);
extern void PushCTXValues(DmtxByteList *valueList, DmtxByte inputValue, int targetScheme, DmtxPassFail *passFail);
extern DmtxBoolean IsCTX(int scheme);
extern void ShiftValueListBy3(DmtxByteList *list, DmtxPassFail *passFail);

/* dmtxencodeedifact.c */
extern void EncodeNextChunkEdifact(DmtxEncodeStream *stream);
extern void AppendValueEdifact(DmtxEncodeStream *stream, DmtxByte value);
extern void CompleteIfDoneEdifact(DmtxEncodeStream *stream, int sizeIdxRequest);

/* dmtxencodebase256.c */
extern void EncodeNextChunkBase256(DmtxEncodeStream *stream);
extern void AppendValueBase256(DmtxEncodeStream *stream, DmtxByte value);
extern void CompleteIfDoneBase256(DmtxEncodeStream *stream, int sizeIdxRequest);
extern void UpdateBase256ChainHeader(DmtxEncodeStream *stream, int perfectSizeIdx);
extern void Base256OutputChainInsertFirst(DmtxEncodeStream *stream);
extern void Base256OutputChainRemoveFirst(DmtxEncodeStream *stream);
extern DmtxByte Randomize255State(DmtxByte cwValue, int cwPosition);
extern unsigned char UnRandomize255State(unsigned char value, int idx);

static const int dmtxNeighborNone = 8;
static const int dmtxPatternX[] = { -1,  0,  1,  1,  1,  0, -1, -1 };
static const int dmtxPatternY[] = { -1, -1, -1,  0,  1,  1,  1,  0 };
static const DmtxPointFlow dmtxBlankEdge = { 0, 0, 0, DmtxUndefined, { -1, -1 } };

/*@ +charint @*/

static int rHvX[] =
    {  256,  256,  256,  256,  255,  255,  255,  254,  254,  253,  252,  251,  250,  249,  248,
       247,  246,  245,  243,  242,  241,  239,  237,  236,  234,  232,  230,  228,  226,  224,
       222,  219,  217,  215,  212,  210,  207,  204,  202,  199,  196,  193,  190,  187,  184,
       181,  178,  175,  171,  168,  165,  161,  158,  154,  150,  147,  143,  139,  136,  132,
       128,  124,  120,  116,  112,  108,  104,  100,   96,   92,   88,   83,   79,   75,   71,
        66,   62,   58,   53,   49,   44,   40,   36,   31,   27,   22,   18,   13,    9,    4,
         0,   -4,   -9,  -13,  -18,  -22,  -27,  -31,  -36,  -40,  -44,  -49,  -53,  -58,  -62,
       -66,  -71,  -75,  -79,  -83,  -88,  -92,  -96, -100, -104, -108, -112, -116, -120, -124,
      -128, -132, -136, -139, -143, -147, -150, -154, -158, -161, -165, -168, -171, -175, -178,
      -181, -184, -187, -190, -193, -196, -199, -202, -204, -207, -210, -212, -215, -217, -219,
      -222, -224, -226, -228, -230, -232, -234, -236, -237, -239, -241, -242, -243, -245, -246,
      -247, -248, -249, -250, -251, -252, -253, -254, -254, -255, -255, -255, -256, -256, -256 };

static int rHvY[] =
    {    0,    4,    9,   13,   18,   22,   27,   31,   36,   40,   44,   49,   53,   58,   62,
        66,   71,   75,   79,   83,   88,   92,   96,  100,  104,  108,  112,  116,  120,  124,
       128,  132,  136,  139,  143,  147,  150,  154,  158,  161,  165,  168,  171,  175,  178,
       181,  184,  187,  190,  193,  196,  199,  202,  204,  207,  210,  212,  215,  217,  219,
       222,  224,  226,  228,  230,  232,  234,  236,  237,  239,  241,  242,  243,  245,  246,
       247,  248,  249,  250,  251,  252,  253,  254,  254,  255,  255,  255,  256,  256,  256,
       256,  256,  256,  256,  255,  255,  255,  254,  254,  253,  252,  251,  250,  249,  248,
       247,  246,  245,  243,  242,  241,  239,  237,  236,  234,  232,  230,  228,  226,  224,
       222,  219,  217,  215,  212,  210,  207,  204,  202,  199,  196,  193,  190,  187,  184,
       181,  178,  175,  171,  168,  165,  161,  158,  154,  150,  147,  143,  139,  136,  132,
       128,  124,  120,  116,  112,  108,  104,  100,   96,   92,   88,   83,   79,   75,   71,
        66,   62,   58,   53,   49,   44,   40,   36,   31,   27,   22,   18,   13,    9,    4 };

/*@ -charint @*/

enum DmtxErrorMessage {
   DmtxErrorUnknown,
   DmtxErrorUnsupportedCharacter,
   DmtxErrorNotOnByteBoundary,
   DmtxErrorIllegalParameterValue,
   DmtxErrorEmptyList,
   DmtxErrorOutOfBounds,
   DmtxErrorMessageTooLarge,
   DmtxErrorCantCompactNonDigits,
   DmtxErrorUnexpectedScheme,
   DmtxErrorIncompleteValueList
};

static char *dmtxErrorMessage[] = {
   "Unknown error",
   "Unsupported character",
   "Not on byte boundary",
   "Illegal parameter value",
   "Encountered empty list",
   "Out of bounds",
   "Message too large",
   "Can't compact non-digits",
   "Encountered unexpected scheme",
   "Encountered incomplete value list"
};

#endif
