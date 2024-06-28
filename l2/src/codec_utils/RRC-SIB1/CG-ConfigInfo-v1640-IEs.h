/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CG_ConfigInfo_v1640_IEs_H_
#define	_CG_ConfigInfo_v1640_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ServCellInfoListMCG_NR_r16;
struct ServCellInfoListMCG_EUTRA_r16;
struct CG_ConfigInfo_v1700_IEs;

/* CG-ConfigInfo-v1640-IEs */
typedef struct CG_ConfigInfo_v1640_IEs {
	struct ServCellInfoListMCG_NR_r16	*servCellInfoListMCG_NR_r16;	/* OPTIONAL */
	struct ServCellInfoListMCG_EUTRA_r16	*servCellInfoListMCG_EUTRA_r16;	/* OPTIONAL */
	struct CG_ConfigInfo_v1700_IEs	*nonCriticalExtension;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CG_ConfigInfo_v1640_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CG_ConfigInfo_v1640_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_CG_ConfigInfo_v1640_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_CG_ConfigInfo_v1640_IEs_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ServCellInfoListMCG-NR-r16.h"
#include "ServCellInfoListMCG-EUTRA-r16.h"
#include "CG-ConfigInfo-v1700-IEs.h"

#endif	/* _CG_ConfigInfo_v1640_IEs_H_ */
#include <asn_internal.h>
