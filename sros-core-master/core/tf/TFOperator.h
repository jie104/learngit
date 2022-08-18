/*
 * TFOperator.h
 *
 *  Created on: 2016年1月24日
 *      Author: lfc
 */

#ifndef TRANSFORM_TFOPERATOR_H_
#define TRANSFORM_TFOPERATOR_H_

#include "TF.h"

namespace slam {
namespace tf {

class TFOperator: public TF {
public:
	//TODO:指针清空
	TFOperator();
	TFOperator(FrameToFrame tf_frame_);
	virtual ~TFOperator();
	bool getTransForm(const int64_t& tf_time, TransForm& output_tf);
	bool lookUpTransForm(const int64_t& tf_time, TransForm& output_tf,
			const int64_t delta_time = 200000); //查看，在当前时间戳附近的0.2秒内是否有可用的tf
	bool lookUpMeanTransForm(const int64_t& tf_time, TransForm& output_tf,int range_index = 4,
			const int64_t delta_time = 200000); //查看，在当前时间戳附近的0.2秒内是否有可用的tf
	bool pushbackTransForm(TransForm& input_tf); //TODO:需要在pushback的时候，查看，pushback的那个，index是否过大，如果过大，就需要删除一部分

	bool findTransForm(const int64_t& tf_time, TransForm& output_tf);

	bool lookUpTransForms(const int64_t& tf_time, std::vector<TransForm>& output_tf,int tfs_size,int64_t delta_time = 200000);

        bool lookUpTransForms(const int64_t& start_tf_time,const int64_t& end_tf_time,  std::vector<TransForm>& output_tf); //查看，在当前时间戳间隔的tf

	void setTfFrame(FrameToFrame tf_frame_);
	void getTfFrame(FrameToFrame& tf_frame_);
	TransFormVector_ptr tf_oprator;
private:
	bool clearTransForm( int max_count,
			int min_count);

	FrameToFrame tf_frame;

};

} /* namespace TF */
} /* namespace SROS */

#endif /* TRANSFORM_TFOPERATOR_H_ */
