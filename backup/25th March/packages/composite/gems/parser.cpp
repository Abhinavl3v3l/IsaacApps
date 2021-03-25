/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "parser.hpp"

#include <algorithm>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/tensor/element_type.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "messages/tensor.hpp"

namespace isaac {
namespace composite {

void Parser::requestSchema(Schema schema) {
  requested_schema_ = std::move(schema);
  setReceivedSchema(std::move(received_schema_));
}

void Parser::parseSchema(::CompositeProto::Reader reader) {
  LOG_DEBUG("1. INSIDE PARSESCHEMA");
  const std::string incoming_schema_hash = reader.getSchemaHash();

  // Do not parse identical schema repeatedly
  if (!incoming_schema_hash.empty() && received_schema_ != std::nullopt
      && received_schema_->getHash() == incoming_schema_hash) {
        LOG_DEBUG("2. INSIDE PARSESCHEMA  FAIL");
    return;
  }

  // Parse received schema
  setReceivedSchema(ReadSchema(reader));
  LOG_DEBUG("1. OUTSIDE PARSESCHEMA");
}

void Parser::setReceivedSchema(std::optional<Schema> schema) {
  LOG_DEBUG("INSIDE setReceivedSchema");
  received_schema_ = std::move(schema);
  LOG_DEBUG("AFTER MOVE");

  if (received_schema_ != std::nullopt) {
    LOG_DEBUG("FRAGMENT INDEX NOT NULL");
    fragment_index_ = FragmentIndex::Create(*received_schema_, requested_schema_,
                                            requested_schema_);
  } else {
    LOG_DEBUG("FRAGMENT INDEX IS NULL");
    fragment_index_ = std::nullopt;
  }
}

bool Parser::parse(::CompositeProto::Reader reader, const std::vector<SharedBuffer>& buffers,
                   double* output_begin, double* output_end) {
  LOG_DEBUG("INSIDE parse 2");
  // Fails on invalid output memory address
  if (output_begin == nullptr) {
     LOG_DEBUG("1 Outputbegin is null");
    return false;
  }
  // Make sure element count matches
  const int output_element_count = output_end - output_begin;
  if (output_element_count != requested_schema_.getElementCount()) {
    LOG_DEBUG("2  Element Count ISSUE");
    return false;
  }
  // No parsing is needed
  if (output_element_count == 0) {
    LOG_DEBUG("2  Element Count ISSUE ZZzzzzzero");
    return true;
  }

  // Parse the incoming schema
  parseSchema(reader);
  // Fail in case of incompatible schema
  if (fragment_index_ == std::nullopt) {
    LOG_DEBUG("3  FRAGMENT INDEX NULL return false");
    return false;
  }

  // Get tensor storing quantity values
  CpuUniversalTensorConstView values;
  if (!FromProto(reader.getValues(), buffers, values)) {
    LOG_DEBUG("4  FROM PROTO FAIL");
    return false;
  }
  if (values.rank() == 0) {
    LOG_DEBUG("5  RANK  ISSUE");
    return false;
  }
  const int values_state_count = values.dimensions()[values.rank() - 1];
  if (values_state_count != received_schema_->getElementCount()) {
    LOG_DEBUG("6  VALUE STATE COUNT  ISSUE");
    LOG_DEBUG("values_state_count is  %i and received_schema_->getElementCount() is %i ",values_state_count,  received_schema_->getElementCount());
    return false;
  }
  LOG_DEBUG("6 OUT");
  if (values.element_type() != ElementType::kFloat64) {
    LOG_DEBUG("7  RANK  ELEMENT TYPE ISSUE");
    return false;
  }
  LOG_DEBUG("8  COPYING  FRAGMENT");
  // Copy values from tensor to output buffer
  const double* values_ptr = reinterpret_cast<const double*>(values.buffer().begin());
  fragment_index_->copyFragments(values_ptr, values_state_count,
                                 output_begin, output_end - output_begin);
  LOG_DEBUG("OUT OF PARSE");
  return true;
}

}  // namespace composite
}  // namespace isaac
