// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <cstdlib>
#include "HandeyeCalibration.h"

namespace TU
{
template <class T> void
doJob(bool single, bool eye_on_hand)
{
    size_t	nposes;
    std::cin >> nposes;
    std::vector<Transform<T> >	cMo(nposes), wMe(nposes);
    for (size_t n = 0; n < nposes; ++n)
    {
	std::cin >> cMo[n] >> wMe[n];
	std::cout << "=== cMo[" << n << "] ===" << std::endl;
	cMo[n].print(std::cout);

	std::cout << "=== wMe[" << n << "] ===" << std::endl;
	if (eye_on_hand)
	    wMe[n].print(std::cout) << std::endl;
	else
	    wMe[n].inverse().print(std::cout) << std::endl;
    }


    const auto	eMc = (single ? cameraToEffectorSingle(cMo, wMe)
			      : cameraToEffectorDual(cMo, wMe));
    const auto	wMo = objectToWorld(cMo, wMe, eMc);
    evaluateAccuracy(std::cout, cMo, wMe, eMc, wMo);
}

}	// namespace TU

int
main(int argc, char* argv[])
{
    bool	single = false, eye_on_hand = false;
    for (int c; (c = getopt(argc, argv, "se")) !=EOF; )
	switch (c)
	{
	  case 's':
	    single = true;
	    break;
	  case 'e':
	    eye_on_hand = true;
	    break;
	}

    TU::doJob<double>(single, eye_on_hand);

    return 0;
}
