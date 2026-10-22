// Compatibility for the Android 16 MiTEE vendor blob, which calls the
// three-argument COSE API removed when cppcose signature creation was refactored.
#include <keymaster/cppcose/cppcose.h>

namespace cppcose {

ErrMsgOr<cppbor::Array> constructCoseSign1(const bytevec& key,
                                           const bytevec& payload,
                                           const bytevec& aad) {
    return constructCoseSign1(key, cppbor::Map{}, payload, aad, EDDSA);
}

}  // namespace cppcose
