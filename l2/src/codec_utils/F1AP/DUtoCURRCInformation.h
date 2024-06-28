/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "F1AP-IEs"
 * 	found in "../F1.asn1"
 * 	`asn1c -D ../F1_output/ -fcompound-names -fno-include-deps -findirect-choice -gen-PER`
 */

#ifndef	_DUtoCURRCInformation_H_
#define	_DUtoCURRCInformation_H_


#include <asn_application.h>

/* Including external dependencies */
#include "CellGroupConfigF1AP.h"
#include "MeasGapConfig.h"
#include <OCTET_STRING.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ProtocolExtensionContainer;

/* DUtoCURRCInformation */
typedef struct DUtoCURRCInformation {
	CellGroupConfigF1AP_t	 cellGroupConfig;
	MeasGapConfig_t	*measGapConfig;	/* OPTIONAL */
	OCTET_STRING_t	*requestedP_MaxFR1;	/* OPTIONAL */
	struct ProtocolExtensionContainer	*iE_Extensions;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DUtoCURRCInformation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DUtoCURRCInformation;

#ifdef __cplusplus
}
#endif

#endif	/* _DUtoCURRCInformation_H_ */
#include <asn_internal.h>
