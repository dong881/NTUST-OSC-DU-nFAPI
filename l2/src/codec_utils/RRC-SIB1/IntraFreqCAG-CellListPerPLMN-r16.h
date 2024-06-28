/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_IntraFreqCAG_CellListPerPLMN_r16_H_
#define	_IntraFreqCAG_CellListPerPLMN_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PCI_Range;

/* IntraFreqCAG-CellListPerPLMN-r16 */
typedef struct IntraFreqCAG_CellListPerPLMN_r16 {
	long	 plmn_IdentityIndex_r16;
	struct IntraFreqCAG_CellListPerPLMN_r16__cag_CellList_r16 {
		A_SEQUENCE_OF(struct PCI_Range) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} cag_CellList_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IntraFreqCAG_CellListPerPLMN_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IntraFreqCAG_CellListPerPLMN_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_IntraFreqCAG_CellListPerPLMN_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_IntraFreqCAG_CellListPerPLMN_r16_1[2];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PCI-Range.h"

#endif	/* _IntraFreqCAG_CellListPerPLMN_r16_H_ */
#include <asn_internal.h>
