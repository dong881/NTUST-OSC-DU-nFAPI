/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_InterRAT_Parameters_H_
#define	_InterRAT_Parameters_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct EUTRA_Parameters;
struct UTRA_FDD_Parameters_r16;

/* InterRAT-Parameters */
typedef struct InterRAT_Parameters {
	struct EUTRA_Parameters	*eutra;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct InterRAT_Parameters__ext1 {
		struct UTRA_FDD_Parameters_r16	*utra_FDD_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} InterRAT_Parameters_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_InterRAT_Parameters;
extern asn_SEQUENCE_specifics_t asn_SPC_InterRAT_Parameters_specs_1;
extern asn_TYPE_member_t asn_MBR_InterRAT_Parameters_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "EUTRA-Parameters.h"
#include "UTRA-FDD-Parameters-r16.h"

#endif	/* _InterRAT_Parameters_H_ */
#include <asn_internal.h>
