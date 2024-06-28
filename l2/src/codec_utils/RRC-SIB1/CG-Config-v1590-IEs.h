/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CG_Config_v1590_IEs_H_
#define	_CG_Config_v1590_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ARFCN-ValueNR.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "ARFCN-ValueEUTRA.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct CG_Config_v1610_IEs;

/* CG-Config-v1590-IEs */
typedef struct CG_Config_v1590_IEs {
	struct CG_Config_v1590_IEs__scellFrequenciesSN_NR {
		A_SEQUENCE_OF(ARFCN_ValueNR_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *scellFrequenciesSN_NR;
	struct CG_Config_v1590_IEs__scellFrequenciesSN_EUTRA {
		A_SEQUENCE_OF(ARFCN_ValueEUTRA_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *scellFrequenciesSN_EUTRA;
	struct CG_Config_v1610_IEs	*nonCriticalExtension;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CG_Config_v1590_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CG_Config_v1590_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_CG_Config_v1590_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_CG_Config_v1590_IEs_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CG-Config-v1610-IEs.h"

#endif	/* _CG_Config_v1590_IEs_H_ */
#include <asn_internal.h>
