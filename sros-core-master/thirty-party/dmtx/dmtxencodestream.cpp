/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2011 Mike Laughton. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact: Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtxencodestream.c
 * \brief DmtxEncodeStream implementation
 */

/**
 *
 *
 */
#include <assert.h>
#include <stdlib.h>
#include "dmtx.h"
#include "dmtxstatic.h"

 DmtxEncodeStream
StreamInit(DmtxByteList *input, DmtxByteList *output)
{
   DmtxEncodeStream stream;

   stream.input = input;
   stream.output = output;

   stream.currentScheme = DmtxSchemeAscii;
   stream.inputNext = 0;
   stream.outputChainValueCount = 0;
   stream.outputChainWordCount = 0;
   stream.reason = NULL;
   stream.sizeIdx = DmtxUndefined;
   stream.status = DmtxStatusEncoding;

   return stream;
}

/**
 *
 *
 */
 void
StreamCopy(DmtxEncodeStream *dst, DmtxEncodeStream *src)
{
   DmtxPassFail passFail;

   dst->currentScheme = src->currentScheme;
   dst->inputNext = src->inputNext;
   dst->outputChainValueCount = src->outputChainValueCount;
   dst->outputChainWordCount = src->outputChainWordCount;
   dst->reason = src->reason;
   dst->sizeIdx = src->sizeIdx;
   dst->status = src->status;
   dst->input = src->input;

   dmtxByteListCopy(dst->output, src->output, &passFail);
}

/**
 *
 *
 */
 void
StreamMarkComplete(DmtxEncodeStream *stream, int sizeIdx)
{
   if(stream->status == DmtxStatusEncoding)
   {
      stream->sizeIdx = sizeIdx;
      stream->status = DmtxStatusComplete;
      assert(stream->reason == NULL);
   }
}

/**
 *
 *
 */
 void
StreamMarkInvalid(DmtxEncodeStream *stream, int reasonIdx)
{
   stream->status = DmtxStatusInvalid;
   stream->reason = dmtxErrorMessage[reasonIdx];
}

/**
 *
 *
 */
 void
StreamMarkFatal(DmtxEncodeStream *stream, int reasonIdx)
{
   stream->status = DmtxStatusFatal;
   stream->reason = dmtxErrorMessage[reasonIdx];
}

/**
 * push on newest/last append
 * used for encoding each output cw
 */
 void
StreamOutputChainAppend(DmtxEncodeStream *stream, DmtxByte value)
{
   DmtxPassFail passFail;

   dmtxByteListPush(stream->output, value, &passFail);

   if(passFail == DmtxPass)
      stream->outputChainWordCount++;
   else
      StreamMarkFatal(stream, DmtxErrorOutOfBounds);
}

/**
 * pop off newest/last
 * used for edifact
 */
 DmtxByte
StreamOutputChainRemoveLast(DmtxEncodeStream *stream)
{
   DmtxByte value;
   DmtxPassFail passFail;

   if(stream->outputChainWordCount > 0)
   {
      value = dmtxByteListPop(stream->output, &passFail);
      stream->outputChainWordCount--;
   }
   else
   {
      value = 0;
      StreamMarkFatal(stream, DmtxErrorEmptyList);
   }

   return value;
}

/**
 * overwrite arbitrary element
 * used for binary length changes
 */
 void
StreamOutputSet(DmtxEncodeStream *stream, int index, DmtxByte value)
{
   if(index < 0 || index >= stream->output->length)
      StreamMarkFatal(stream, DmtxErrorOutOfBounds);
   else
      stream->output->b[index] = value;
}

/**
 *
 *
 */
 DmtxBoolean
StreamInputHasNext(DmtxEncodeStream *stream)
{
   return (stream->inputNext < stream->input->length) ? DmtxTrue : DmtxFalse;
}

/**
 * peek at first/oldest
 * used for ascii double digit
 */
 DmtxByte
StreamInputPeekNext(DmtxEncodeStream *stream)
{
   DmtxByte value = 0;

   if(StreamInputHasNext(stream))
      value = stream->input->b[stream->inputNext];
   else
      StreamMarkFatal(stream, DmtxErrorOutOfBounds);

   return value;
}

/**
 * used as each input cw is processed
 *
 * \param value Value to populate, can be null (for blind dequeues)
 * \param stream
 */
 DmtxByte
StreamInputAdvanceNext(DmtxEncodeStream *stream)
{
   DmtxByte value;

   value = StreamInputPeekNext(stream);

   if(stream->status == DmtxStatusEncoding)
      stream->inputNext++; /* XXX is this what we really mean here? */

   return value;
}

/**
 * used as each input cw is processed
 *
 * \param value Value to populate, can be null (for blind dequeues)
 * \param stream
 */
 void
StreamInputAdvancePrev(DmtxEncodeStream *stream)
{
   if(stream->inputNext > 0)
      stream->inputNext--;
   else
      StreamMarkFatal(stream, DmtxErrorOutOfBounds);
}
