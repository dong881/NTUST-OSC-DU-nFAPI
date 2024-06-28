/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "PUCCH-PathlossReferenceRS.h"

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_referenceSignal_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static asn_TYPE_member_t asn_MBR_referenceSignal_3[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PUCCH_PathlossReferenceRS__referenceSignal, choice.ssb_Index),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SSB_Index,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"ssb-Index"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PUCCH_PathlossReferenceRS__referenceSignal, choice.csi_RS_Index),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NZP_CSI_RS_ResourceId,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"csi-RS-Index"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_referenceSignal_tag2el_3[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* ssb-Index */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* csi-RS-Index */
};
static asn_CHOICE_specifics_t asn_SPC_referenceSignal_specs_3 = {
	sizeof(struct PUCCH_PathlossReferenceRS__referenceSignal),
	offsetof(struct PUCCH_PathlossReferenceRS__referenceSignal, _asn_ctx),
	offsetof(struct PUCCH_PathlossReferenceRS__referenceSignal, present),
	sizeof(((struct PUCCH_PathlossReferenceRS__referenceSignal *)0)->present),
	asn_MAP_referenceSignal_tag2el_3,
	2,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_referenceSignal_3 = {
	"referenceSignal",
	"referenceSignal",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_referenceSignal_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		CHOICE_constraint
	},
	asn_MBR_referenceSignal_3,
	2,	/* Elements count */
	&asn_SPC_referenceSignal_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_PUCCH_PathlossReferenceRS_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PUCCH_PathlossReferenceRS, pucch_PathlossReferenceRS_Id),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PUCCH_PathlossReferenceRS_Id,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"pucch-PathlossReferenceRS-Id"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PUCCH_PathlossReferenceRS, referenceSignal),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_referenceSignal_3,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"referenceSignal"
		},
};
static const ber_tlv_tag_t asn_DEF_PUCCH_PathlossReferenceRS_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PUCCH_PathlossReferenceRS_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* pucch-PathlossReferenceRS-Id */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* referenceSignal */
};
asn_SEQUENCE_specifics_t asn_SPC_PUCCH_PathlossReferenceRS_specs_1 = {
	sizeof(struct PUCCH_PathlossReferenceRS),
	offsetof(struct PUCCH_PathlossReferenceRS, _asn_ctx),
	asn_MAP_PUCCH_PathlossReferenceRS_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_PUCCH_PathlossReferenceRS = {
	"PUCCH-PathlossReferenceRS",
	"PUCCH-PathlossReferenceRS",
	&asn_OP_SEQUENCE,
	asn_DEF_PUCCH_PathlossReferenceRS_tags_1,
	sizeof(asn_DEF_PUCCH_PathlossReferenceRS_tags_1)
		/sizeof(asn_DEF_PUCCH_PathlossReferenceRS_tags_1[0]), /* 1 */
	asn_DEF_PUCCH_PathlossReferenceRS_tags_1,	/* Same as above */
	sizeof(asn_DEF_PUCCH_PathlossReferenceRS_tags_1)
		/sizeof(asn_DEF_PUCCH_PathlossReferenceRS_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_PUCCH_PathlossReferenceRS_1,
	2,	/* Elements count */
	&asn_SPC_PUCCH_PathlossReferenceRS_specs_1	/* Additional specs */
};

