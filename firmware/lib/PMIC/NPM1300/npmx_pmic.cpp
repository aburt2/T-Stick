#include "npmx_pmic.hpp"

npmx_pmic::npmx_pmic(NPM1300_PMIC &_npmx_class) {
    _npmx = &_npmx_class;
}


void npmx_pmic::begin() {
    // Initialise backend
    _npmx->begin();
}

void npmx_pmic::begin(NPM1300_PMIC &_npmx_class) {
    _npmx = &_npmx_class;
    begin();
}
