// SPDX-License-Identifier: MIT-0

/**
 * @file
 * Provides an example of creating a simple Drake C++ system that will later be
 * bound.
 *
 * This is the instantiation portion of the -inl.h pattern:
 * http://drake.mit.edu/cxx_inl.html#cxx-inl-files
 */

#include "simple_adder-inl.h"

#include <drake/common/default_scalars.h>

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake_external_examples::SimpleAdder)
