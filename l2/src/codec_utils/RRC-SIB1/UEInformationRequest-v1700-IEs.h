/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UEInformationRequest_v1700_IEs_H_
#define	_UEInformationRequest_v1700_IEs_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UEInformationRequest_v1700_IEs__successHO_ReportReq_r17 {
	UEInformationRequest_v1700_IEs__successHO_ReportReq_r17_true	= 0
} e_UEInformationRequest_v1700_IEs__successHO_ReportReq_r17;
typedef enum UEInformationRequest_v1700_IEs__coarseLocationRequest_r17 {
	UEInformationRequest_v1700_IEs__coarseLocationRequest_r17_true	= 0
} e_UEInformationRequest_v1700_IEs__coarseLocationRequest_r17;

/* UEInformationRequest-v1700-IEs */
typedef struct UEInformationRequest_v1700_IEs {
	long	*successHO_ReportReq_r17;	/* OPTIONAL */
	long	*coarseLocationRequest_r17;	/* OPTIONAL */
	struct UEInformationRequest_v1700_IEs__nonCriticalExtension {
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *nonCriticalExtension;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UEInformationRequest_v1700_IEs_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_successHO_ReportReq_r17_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_coarseLocationRequest_r17_4;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_UEInformationRequest_v1700_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_UEInformationRequest_v1700_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_UEInformationRequest_v1700_IEs_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _UEInformationRequest_v1700_IEs_H_ */
#include <asn_internal.h>
