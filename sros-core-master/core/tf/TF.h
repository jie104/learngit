/*
 * TF.h
 *
 *  Created on: 2016年1月23日
 *      Author: lfc
 */
#include <boost/shared_ptr.hpp>
#include "CircleArray.hpp"
#include "TransForm.h"
#include "map"
#include "string"
#include "vector"
#ifndef TRANSFORM_TF_H_
#define TRANSFORM_TF_H_
#define TF_MAX_COUNT 2000
//#define TF_MAX_COUNT 1024
typedef std::string Tf_Frame_name;
namespace slam {
namespace tf {
typedef std::shared_ptr<CircleArray<TransForm>> TransFormVector_ptr;
typedef CircleArray<TransForm> TransFormVector;
enum Tf_Vector_Size {
    MIN_SIZE = 1024,
    MAX_SIZE = 2048,
};
struct FrameToFrame {
    std::string parent_frame;
    std::string child_frame;
};
Tf_Frame_name encodeTfFrame(FrameToFrame tf_name);
FrameToFrame decodeParentChildFrame(Tf_Frame_name tf_name);
class TF {
public:
	TF();
	virtual ~TF();
	bool getTransForm(const int64_t& tf_time, FrameToFrame tf_name,
					  TransForm& output_tf);
	bool lookUpTransForm(const int64_t& tf_time, FrameToFrame tf_name,
						 int64_t delta_time = 200000); //查看，在当前时间戳附近的0.2秒内是否有可用的tf
	bool lookUpTransForm(const int64_t& tf_time, FrameToFrame tf_name,
						 TransForm& output_tf, int64_t delta_time = 200000); //查看，在当前时间戳附近的0.2秒内是否有可用的tf
	bool pushbackTransForm(FrameToFrame tf_name_frame, TransForm& input_tf); //TODO:需要在pushback的时候，查看，pushback的那个，index是否过大，如果过大，就需要删除一部分

	bool findTransForm(const int64_t& tf_time, FrameToFrame tf_name_frame,
					   TransForm& output_tf);
	bool findinTimeTF(const int64_t& tf_time, TransFormVector_ptr& transform_vector,TransForm& output_tf);

	bool findinTimeMeanTF(const int64_t& tf_time, TransFormVector_ptr& transform_vector,TransForm& output_tf,int range_index);

        bool findinTimeTFs(const int64_t& tf_time, TransFormVector_ptr& transform_vector,std::vector<TransForm>& output_tf,int tfs_size);
	static std::map<Tf_Frame_name, TransFormVector_ptr> tf_vector;
private:
	bool clearTransForm(FrameToFrame tf_name_frame, int max_count,
						int min_count);

};
}  // namespace tf
}  // namespace slam

#endif /* TRANSFORM_TF_H_ */
